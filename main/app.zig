const std = @import("std");
const builtin = @import("builtin");
const idf = @import("esp_idf");
const ver = idf.ver.Version;
const mem = std.mem;

const uzip = @import("uzip");

const ads1115 = @import("ads1115.zig");
const Ads1115 = ads1115.ADS1115Driver();

const cfg = @import("app_config");

comptime {
    if (cfg.wifi_ssid.len == 0)
        @compileError("no WiFi SSID — build with -Dssid=… (zig) or -DWIFI_SSID=… (idf.py)");
    if (cfg.wifi_password.len == 0)
        @compileError("no WiFi password — build with -Dpassword=… (zig) or -DWIFI_PASSWORD=… (idf.py)");
}

fn padArray(comptime N: usize, comptime s: []const u8) [N]u8 {
    if (s.len > N)
        @compileError("value exceeds " ++ std.fmt.comptimePrint("{d}", .{N}) ++ "-byte limit");
    var tmp: [N]u8 = @splat(0);
    @memcpy(tmp[0..s.len], s);
    return tmp;
}

comptime {
    @export(&main, .{ .name = "app_main" });
}

const RX_QUEUE_SLOTS: u8 = 8;
const MAX_FRAME_SIZE: usize = 1536;

const RxFrame = struct {
    data: [MAX_FRAME_SIZE]u8 = .{0} ** MAX_FRAME_SIZE,
    len: u16 = 0,
};

const Stack = uzip.core.stack.NetworkStack(1, 8, .{
    .enable_arp = true,
    .enable_ipv4 = true,
    .enable_icmp = true,
    .enable_udp = true,
    .enable_tcp = true,
    .millis = millis,
});

fn millis() u32 {
    return @intCast(@divTrunc(idf.timer.Timer.getTime(), std.time.us_per_ms));
}

var stack: Stack = .{};
var rx_queue: idf.rtos.Queue.Handle = null;
var rx_frame: RxFrame = .{};

var dhcp_client: uzip.app.dhcp.DhcpClient = .{};

const I2cMasterBusConfig = extern struct {
    i2c_port: c_int = -1,
    sda_io_num: c_int,
    scl_io_num: c_int,
    clk_source: i32 = idf.sys.I2C_CLK_SRC_DEFAULT,
    glitch_ignore_cnt: u8 = 7,
    intr_priority: c_int = 0,
    trans_queue_depth: usize = 0,
    flags: extern struct {
        enable_internal_pullup: u32 = 1,
        allow_pd: u32 = 0,
    } = .{},
};

const I2cDeviceConfig = extern struct {
    dev_addr_length: c_uint = idf.sys.I2C_ADDR_BIT_LEN_7,
    device_address: u16,
    scl_speed_hz: u32 = 100_000,
    scl_wait_us: u32 = 0,
    flags: extern struct {
        disable_ack_check: u32 = 0,
    } = .{},
};

fn i2cTransmit(ctx: *anyopaque, buf: []const u8, timeout_ms: i32) !void {
    return idf.i2c.transmit(@ptrCast(ctx), buf, timeout_ms);
}
fn i2cReceive(ctx: *anyopaque, buf: []u8, timeout_ms: i32) !void {
    return idf.i2c.receive(@ptrCast(ctx), buf, timeout_ms);
}
fn i2cTransmitReceive(ctx: *anyopaque, tx: []const u8, rx: []u8, timeout_ms: i32) !void {
    return idf.i2c.transmitReceive(@ptrCast(ctx), tx, rx, timeout_ms);
}

fn readAllAdcPins(ads: *Ads1115) ![4]u16 {
    var raw_adc: [4]u16 = undefined;
    for (0..4) |i| {
        try ads.writeConfig(.{
            .mode = .SingleShot,
            .mux = @enumFromInt(@as(u3, @intFromEnum(ads1115.AdcMux.AIN0)) + @as(u3, @intCast(i))),
            .pga = .FSL_6_144V,
            .os = 1,
            .dr = .@"250SPS",
        });
        idf.rtos.Task.delayMs(10);

        raw_adc[i] = try ads.readAdc();
    }
    return raw_adc;
}

fn main() callconv(.c) void {
    const i2c_bus_cfg = I2cMasterBusConfig{
        .sda_io_num = 6,
        .scl_io_num = 7,
        .flags = .{
            .enable_internal_pullup = 0,
        },
    };

    var i2c_bus_handle: idf.sys.i2c_master_bus_handle_t = null;
    idf.i2c.BUS.add(@ptrCast(&i2c_bus_cfg), &i2c_bus_handle) catch |err| {
        log.err("I2C bus init failed: {s}", .{@errorName(err)});
    };

    // var found: u32 = 0;
    // log.info("Scanning...", .{});

    const i2c_addr: u16 = 0b1001000;
    const i2c_dev_cfg = I2cDeviceConfig{
        .device_address = i2c_addr,
    };
    var i2c_dev_handle: idf.sys.i2c_master_dev_handle_t = null;
    idf.i2c.BUS.addDevice(i2c_bus_handle, @ptrCast(&i2c_dev_cfg), &i2c_dev_handle) catch |err| {
        log.err("I2C: addDevice failed: {s}", .{@errorName(err)});
    };

    var adc = Ads1115{
        .context = @ptrCast(i2c_dev_handle),
        .i2cTransmit = i2cTransmit,
        .i2cReceive = i2cReceive,
        .i2cTransmitReceive = i2cTransmitReceive,
    };

    adc.writeConfig(.{
        .mux = .AIN0,
        .pga = .FSL_6_144V,
        .mode = .SingleShot,
        .dr = .@"250SPS",
        .os = 1,
    }) catch |err| {
        log.err("ADS1115 writeConfig failed: {s}", .{@errorName(err)});
    };

    const readback = adc.readConfig() catch |err| {
        log.err("ADS1115 readConfig failed: {s}", .{@errorName(err)});
        return;
    };
    log.info("CONFIG: 0x{X:02}", .{@as(u16, @bitCast(readback))});
    log.info("ADS1115 config: mode={s} os={d} dr={s} mux={s}", .{
        @tagName(readback.mode),
        readback.os,
        @tagName(readback.dr),
        @tagName(readback.mux),
    });

    // while (true) {
    //     idf.rtos.Task.delayMs(1000);
    //     const raw = adc.readAdcDirect() catch |err| {
    //         log.err("ADS1115 readAdc failed: {s}", .{@errorName(err)});
    //         return;
    //     };
    //     log.info("ADS1115 raw: {d}", .{raw});
    // }
    // while (true) {
    //     idf.rtos.Task.delayMs(1000);

    //     adc.setPointer(.Conversion) catch {};

    //     var rx_buf: [2]u8 = undefined;
    //     adc.i2cReceive(adc.context, &rx_buf, 100) catch {};

    //     const raw = std.mem.bigToNative(
    //         u16,
    //         @as(u16, @bitCast(rx_buf)),
    //     );

    //     log.info("ADS1115 raw: {d}", .{raw});
    // }
    // adc.setAddrPointer(.Conversion) catch {};

    // while (true) {
    //     var rx_buf: [2]u8 = undefined;

    //     adc.i2cReceive(adc.context, &rx_buf, 100) catch {};

    //     const raw = std.mem.bigToNative(
    //         u16,
    //         @as(u16, @bitCast(rx_buf)),
    //     );

    //     log.info("ADS1115 raw: {d}", .{raw});

    //     idf.rtos.Task.delayMs(10);
    // }
    // This allocator is safe to use as the backing allocator w/ arena allocator

    // custom allocators (based on old raw_c_allocator)
    // idf.heap.HeapCapsAllocator
    // idf.heap.MultiHeapAllocator
    // idf.heap.VPortAllocator

    idf.sys.esp_log_level_set("logging", idf.sys.ESP_LOG_VERBOSE);

    var heap = idf.heap.HeapCapsAllocator.init(.{ .@"8bit" = true });
    var arena = std.heap.ArenaAllocator.init(heap.allocator());
    defer arena.deinit();
    const allocator = arena.allocator();

    log.info("Hello, world from Zig!", .{});

    log.info(
        \\[Zig Info]
        \\* Version: {s}
        \\* Compiler Backend: {s}
    , .{
        @as([]const u8, builtin.zig_version_string),
        @tagName(builtin.zig_backend),
    });

    log.info(
        \\[ESP-IDF Info]
        \\* Version: {s}
    , .{ver.get().toString(allocator)});

    stack.init();

    idf.nvs.flashInitOrErase() catch @panic("ruh roh");

    idf.event.loopCreateDefault() catch @panic("ruh roh2");

    const wifi_init_cfg = idf.wifi.init_config_default();

    idf.wifi.init(&wifi_init_cfg) catch @panic("ruh roh3");

    idf.event.handlerRegister(idf.sys.WIFI_EVENT, idf.event.ANY_ID, onWifiEvent, null) catch @panic("ruh roh4");

    idf.wifi.setMode(.WIFI_MODE_STA) catch @panic("ruh roh5");

    var wifi_cfg: idf.wifi.wifiConfig = .{
        .sta = .{
            .ssid = padArray(32, cfg.wifi_ssid),
            .password = padArray(64, cfg.wifi_password),
        },
    };

    idf.wifi.setConfig(.WIFI_IF_STA, &wifi_cfg) catch @panic("ruh roh4");

    idf.wifi.start() catch @panic("ruh roh5");

    idf.wifi.PowerSave.set(idf.sys.WIFI_PS_NONE) catch @panic("ruh roh6");

    var mac_buf: [6:0]u8 = undefined;
    idf.wifi.MAC.get(.WIFI_IF_STA, &mac_buf) catch @panic("ruh roh7");
    @memcpy(&stack.interfaces[0].mac_addr, &mac_buf);

    stack.interfaces[0].device = .{ .eth = .{
        .transmit = uzipTransmit,
        .poll_recv = uzipPollRecv,
    } };

    dhcp_client.init(&stack.interfaces[0]) catch @panic("ruh roh 7.5");
    _ = uzip.app.http.init(handleRequest, null);

    rx_queue = idf.rtos.Queue.create(RX_QUEUE_SLOTS, @sizeOf(RxFrame)) catch @panic("ruh roh8");
    _ = idf.rtos.Task.create(uzipTask, "uzip", 32 * 1024, null, 5) catch @panic("ruh roh9");

    // FreeRTOS Tasks — Task.create returns !Handle; on failure panic with a clear message.
    // _ = idf.rtos.Task.create(fooTask, "foo", 1024 * 4, null, 1) catch @panic("Task foo not created");
    // _ = idf.rtos.Task.create(barTask, "bar", 1024 * 4, null, 2) catch @panic("Task bar not created");
    // _ = idf.rtos.Task.create(blinkTask, "blink", 1024 * 2, null, 5) catch @panic("Task blink not created");

    while (true) {
        idf.rtos.Task.delayMs(1000);

        const values = readAllAdcPins(&adc) catch .{ 0, 0, 0, 0 };
        const t0 = 6.144 * @as(f32, @floatFromInt(values[0])) / std.math.maxInt(u15);
        const t1 = 6.144 * @as(f32, @floatFromInt(values[1])) / std.math.maxInt(u15);
        const t2 = 6.144 * @as(f32, @floatFromInt(values[2])) / std.math.maxInt(u15);
        const t3 = 6.144 * @as(f32, @floatFromInt(values[3])) / std.math.maxInt(u15);
        grillmon_status.temperatures = .{ t0, t1, t2, t3 };
        var rssi: i32 = 0;
        idf.wifi.Station.getRssi(&rssi) catch @panic("ruh roh10");
        grillmon_status.rssi = @intCast(rssi);
        grillmon_status.uptime = millis() / 1000;
    }
}

fn uzipPollRecv() ?[]u8 {
    if (rx_queue == null) return null;
    if (!idf.rtos.Queue.receive(rx_queue, &rx_frame, 0)) return null;
    return rx_frame.data[0..rx_frame.len];
}

const GrillmonBinary = extern struct {
    protocol_version: u8 align(1) = 0,
    flags: u8 align(1) = 0,
    time_start: u32 align(1) = 0,
    temperatures: [4]f32 align(1),
    grill_set_point: i16 align(1) = 0,
    battery: u8 align(1) = 0,
    rssi: i16 align(1) = 0,
    battery_voltage: u16 align(1) = 0,
    uptime: u32 align(1) = 0,
};

// Static so the body slice stays valid across async chunked sends.
var grillmon_status = GrillmonBinary{
    .flags = 0b00011110,
    .protocol_version = 1,
    .battery = 80,
    .grill_set_point = 225,
    .temperatures = .{ 0, 0, 0, 0 },
    .rssi = -50,
    .time_start = 3600,
};

const index_html = @embedFile("grillmon.html");

fn handleRequest(req: *const uzip.app.http.Request, ctx: ?*anyopaque) anyerror!uzip.app.http.Response {
    _ = ctx;
    switch (req.method) {
        .GET => {
            if (std.ascii.eqlIgnoreCase(req.path(), "/")) {
                return .{
                    .body = index_html,
                    .content_length = index_html.len,
                };
            } else if (std.ascii.eqlIgnoreCase(req.path(), "/api/status")) {
                return .{
                    .content_type = "application/octet-stream",
                    .body = std.mem.asBytes(&grillmon_status),
                    .content_length = @sizeOf(GrillmonBinary),
                };
            }
            return error.NotFound;
        },
        .POST => return error.NotFound,
    }
}

fn uzipTransmit(buf: []const u8) void {
    idf.wifi.Internal.txBuffer(.WIFI_IF_STA, @ptrCast(@constCast(buf.ptr)), @intCast(buf.len)) catch |err| {
        log.err("uzip TX failed: {s}", .{@errorName(err)});
    };
}

export fn uzipTask(_: ?*anyopaque) callconv(.c) void {
    var dhcp_timer: u32 = 0;
    while (true) {
        stack.poll();
        idf.rtos.Task.delayMs(1);
        const now = millis();
        if (now - dhcp_timer > 1000) {
            dhcp_timer = now;
            _ = dhcp_client.poll();
        }
    }
}

pub const panic = idf.esp_panic.panic;
const log = std.log.scoped(idf.log.default_log_scope);
pub const std_options: std.Options = .{
    .log_level = switch (builtin.mode) {
        .Debug => .debug,
        else => .info,
    },
    .logFn = idf.log.espLogFn,
    .log_scope_levels = &.{
        .{ .scope = .arp, .level = .err },
        .{ .scope = .eth, .level = .err },
        .{ .scope = .dhcp, .level = .debug },
        .{ .scope = .udp, .level = .err },
    },
};

export fn onWifiEvent(_: ?*anyopaque, event_base: idf.sys.esp_event_base_t, event_id: i32, _: ?*anyopaque) callconv(.c) void {
    _ = event_base;
    switch (event_id) {
        idf.sys.WIFI_EVENT_STA_START => {
            log.info("STA started, connecting...", .{});
            idf.wifi.connect() catch |err|
                log.err("connect() failed: {s}", .{@errorName(err)});
        },
        idf.sys.WIFI_EVENT_STA_DISCONNECTED => {
            idf.wifi.connect() catch |err|
                log.err("connect() failed: {s}", .{@errorName(err)});
            // if (g_retry_count < MAX_RETRY_ATTEMPTS) {
            //     g_retry_count += 1;
            //     log.warn("Disconnected, retry {}/{}", .{ g_retry_count, MAX_RETRY_ATTEMPTS });
            //     idf.wifi.connect() catch {};
            // } else {
            //     _ = sys.xEventGroupSetBits(g_event_group, FAILED_BIT);
            // }
        },
        idf.sys.WIFI_EVENT_STA_CONNECTED => {
            log.info("Wi-Fi successfully connected!!", .{});
            idf.wifi.Internal.registryTXCallBack(.WIFI_IF_STA, wifiRxCallback) catch |err| {
                log.err("failed to register callback: {s}", .{@errorName(err)});
            };
        },
        else => {},
    }
}

var total_bytes_rxd: usize = 0;
export fn wifiRxCallback(buffer_ptr: ?*anyopaque, len: u16, eb: ?*anyopaque) callconv(.c) idf.sys.esp_err_t {
    // log.debug("WIFI RX Callback", .{});
    if (buffer_ptr) |ptr| {
        const buffer = @as([*]u8, @ptrCast(ptr))[0..len];
        total_bytes_rxd += buffer.len;
        // log.info("Frame received: len: {d}:{d}", .{ buffer.len, total_bytes_rxd });

        var item: RxFrame = .{};
        const copy_len = @min(buffer.len, item.data.len);
        @memcpy(item.data[0..copy_len], buffer[0..copy_len]);
        item.len = @intCast(copy_len);

        if (rx_queue != null) {
            if (!idf.rtos.Queue.send(rx_queue, &item, 0)) {
                log.warn("uzip RX queue full, dropping {d} bytes", .{copy_len});
            }
        }
    }
    idf.wifi.Internal.freeRXBuffer(eb) catch {};
    return 0;
}
