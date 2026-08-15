const std = @import("std");

const AddressPointerRegister = packed struct(u8) {
    reg: Registers,
    resv: u6 = 0,
};

const Registers = enum(u2) {
    Conversion = 0,
    Config = 1,
    Lo_thres = 2,
    Hi_thres = 3,
};

pub const Config = packed struct(u16) {
    comp_que: u2 = 0b11,
    comp_lat: u1 = 0,
    comp_pol: u1 = 0,
    comp_mode: u1 = 0,
    dr: DataRate = .@"128SPS",
    mode: Mode,
    pga: Gain,
    mux: AdcMux,
    os: u1,
};

pub const AdcMux = enum(u3) {
    AIN01 = 0b000,
    AIN03 = 0b001,
    AIN13 = 0b010,
    AIN23 = 0b011,
    AIN0 = 0b100,
    AIN1 = 0b101,
    AIN2 = 0b110,
    AIN3 = 0b111,
};

pub const Mode = enum(u1) {
    SingleShot = 0,
    Continuous = 1,
};

pub const Gain = enum(u3) {
    FSL_6_144V = 0b000,
    FSL_4_096V = 0b001,
    FSL_2_048V = 0b010,
    FSL_1_024V = 0b011,
    FSL_0_512V = 0b100,
    FSL_0_256V = 0b101,
};

pub const DataRate = enum(u3) {
    @"8SPS" = 0b000,
    @"16SPS" = 0b001,
    @"32SPS" = 0b010,
    @"64SPS" = 0b011,
    @"128SPS" = 0b100,
    @"250SPS" = 0b101,
    @"475SPS" = 0b110,
    @"860SPS" = 0b111,
};

pub fn ADS1115Driver() type {
    return struct {
        const Self = @This();

        context: *anyopaque,
        i2cTransmit: *const fn (ctx: *anyopaque, buffer: []const u8, timeout_ms: i32) anyerror!void,
        i2cReceive: *const fn (ctx: *anyopaque, buffer: []u8, timeout_ms: i32) anyerror!void,
        i2cTransmitReceive: *const fn (ctx: *anyopaque, tx_buffer: []const u8, rx_buffer: []u8, timeout_ms: i32) anyerror!void,

        pub fn writeConfig(s: *Self, cfg: Config) !void {
            var buf: [3]u8 = undefined;
            buf[0] = @bitCast(AddressPointerRegister{ .reg = .Config });
            const cfg_ordered: u16 = std.mem.nativeToBig(u16, @as(u16, @bitCast(cfg)));
            @memcpy(buf[1..], std.mem.asBytes(&cfg_ordered));
            try s.i2cTransmit(s.context, &buf, 100);
        }

        pub fn setAddrPointer(s: *Self, reg: Registers) !void {
            const buf = [_]u8{@bitCast(AddressPointerRegister{ .reg = reg })};
            try s.i2cTransmit(s.context, &buf, 100);
        }

        pub fn readAdc(s: *Self) !u16 {
            const ptr = [_]u8{@bitCast(AddressPointerRegister{ .reg = .Conversion })};
            var rx_buf: [2]u8 = undefined;
            try s.i2cTransmitReceive(s.context, &ptr, &rx_buf, 100);
            return std.mem.bigToNative(u16, @as(u16, @bitCast(rx_buf)));
        }

        pub fn readAdcDirect(s: *Self) !u16 {
            var rx_buf: [2]u8 = undefined;
            try s.i2cReceive(s.context, &rx_buf, 100);
            const raw = std.mem.bigToNative(u16, @as(u16, @bitCast(rx_buf)));
            return raw;
        }

        pub fn readConfig(s: *Self) !Config {
            const ptr = [_]u8{@bitCast(AddressPointerRegister{ .reg = .Config })};
            var rx_buf: [2]u8 = undefined;
            try s.i2cTransmitReceive(s.context, &ptr, &rx_buf, 100);
            const raw = std.mem.bigToNative(u16, @as(u16, @bitCast(rx_buf)));
            return @bitCast(raw);
        }
    };
}
