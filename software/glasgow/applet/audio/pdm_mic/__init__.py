import argparse
import logging

from amaranth import *
from amaranth.lib import io
from amaranth.lib.cdc import FFSynchronizer

from ....gateware.clockgen import *
from ... import *


class FIFOStreamer(Elaboratable):
    def __init__(self, width, fifo):
        assert width > 0 and width <= 7
        self.width = width
        self.data = Signal(width)
        self.strobe = Signal()
        self.fifo = fifo

    def elaborate(self, platform):
        m = Module()

        with m.FSM():
            with m.State("IDLE"):
                with m.If(self.strobe):
                    with m.If(self.fifo.w_rdy):
                        m.d.comb += [
                            self.fifo.w_data.eq(self.data),
                            self.fifo.w_en.eq(1),
                        ]
                        m.next = "WAIT"
                    with m.Else():
                        m.next = "REPORT_OVERRUN"

            with m.State("REPORT_OVERRUN"):
                with m.If(self.fifo.w_rdy):
                    m.d.comb += [
                        self.fifo.w_data.eq(0xff),
                        self.fifo.w_en.eq(1),
                    ]
                    m.next = "IDLE"

            with m.State("WAIT"):
                m.next = "IDLE"

        return m


class PDMCaptureSubtarget(Elaboratable):
    def __init__(self, ports, in_fifo, sample_cyc):
        self.ports = ports
        self.in_fifo = in_fifo
        self.sample_cyc = sample_cyc

    def elaborate(self, platform):
        m = Module()

        n = len(self.ports.i)

        m.submodules.clk_buffer = clk_buffer = io.Buffer("o", self.ports.clk)
        m.submodules.i_buffer = i_buffer = io.Buffer("i", self.ports.i)

        m.submodules.clkgen = clkgen = ClockGen(self.sample_cyc)
        m.d.comb += clk_buffer.o.eq(clkgen.clk)

        pins_i = Signal.like(i_buffer.i)
        m.submodules += FFSynchronizer(i_buffer.i, pins_i)

        counter = Signal(n * 2)
        counter_prev = Signal(n * 2)

        pins_current = Signal(n * 2)
        channels = Signal(n * 2)

        m.submodules.streamer = streamer = FIFOStreamer(n * 2, self.in_fifo)

        m.d.comb += [
            streamer.data.eq(channels),
            streamer.strobe.eq(clkgen.stb_f)
        ]

        with m.FSM() as fsm:
            with m.State("REPORT"):
                m.d.sync += [
                    channels.eq(Cat(pins_current[0], pins_current[3], pins_current[1], pins_current[4], pins_current[2], pins_current[5])),
                    counter_prev.eq(counter),
                ]
                m.next = "WAIT_RISING_EDGE"

            with m.State("WAIT_RISING_EDGE"):
                with m.If(clkgen.stb_r):
                    m.d.sync += pins_current[:n].eq(pins_i)
                    m.next = "WAIT_FALLING_EDGE"

            with m.State("WAIT_FALLING_EDGE"):
                with m.If(clkgen.stb_f):
                    m.d.sync += pins_current[n:].eq(pins_i)
                    m.d.sync += counter.eq(counter + 1)
                    m.next = "REPORT"

        return m


class AudioPDMApplet(GlasgowApplet):
    logger = logging.getLogger(__name__)
    help = "capture audio using a PDM microphone"
    description = """
    """

    @classmethod
    def add_build_arguments(cls, parser, access):
        super().add_build_arguments(parser, access)

        access.add_pin_argument(parser, "clk", default=True)
        access.add_pin_set_argument(parser, "i", width=3, default=True)

    @classmethod
    def add_interact_arguments(cls, parser):
        parser.add_argument(
            "file", type=argparse.FileType("wb"),
            help="write audio samples to the FILE")

    def build(self, target, args):
        self.mux_interface = iface = target.multiplexer.claim_interface(self, args)

        logging.info(f"target.sys_clk_freq: {target.sys_clk_freq}")
        sample_cyc = self.derive_clock(clock_name="sampling",
            input_hz=target.sys_clk_freq, output_hz=2.2 * 1e6)
        logging.info(f"sample_cyc: {sample_cyc}")
        logging.info(f"sample_cyc_hz: {target.sys_clk_freq / sample_cyc}")

        iface.add_subtarget(PDMCaptureSubtarget(
            ports=iface.get_port_group(
                clk=args.pin_clk,
                i=args.pin_set_i,
            ),
            in_fifo=iface.get_in_fifo(depth=8192),
            sample_cyc=sample_cyc,
        ))

        self._sample_rate_cyc = sample_cyc

    async def interact(self, device, args, iface):
        f = open(args.file.name, "wb")
        logging.info("starting capture")
        total_read = 0
        try:
            overrun = False
            buffer = bytes()
            while not overrun:
                data = await iface.read()
                if len(data) == 0:
                   continue

                data = bytes(data)
                total_read += len(data)

                if 0xff in data:
                    self.logger.error("FIFO overrun, shutting down")
                    overrun = True
                    break

                buffer += data

                if len(buffer) > 1024 * 1024:
                    f.write(buffer)
                    buffer = bytes()

            if len(buffer) > 0:
                f.write(buffer)
                buffer = bytes()
        except Exception as e:
            self.logger.error("Error writing to file: %s", e)
        finally:
            f.close()


    async def run(self, device, args):
        iface = await device.demultiplexer.claim_interface(self, self.mux_interface, args)
        return iface
