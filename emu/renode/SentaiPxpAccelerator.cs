//
// SentAI emulator-only PXP semantic accelerator.
//
// This is intentionally not a full NXP PXP model. It implements the narrow
// resize/format boundary used by sentai_pxp_shim_emu.c so Renode can perform
// large frame transforms on the host side while the guest keeps the normal
// PrepTask/PXP contract.
//

using System;
using System.Diagnostics;
using System.IO;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals
{
    public class SentaiPxpAccelerator : IDoubleWordPeripheral, IKnownSize
    {
        public SentaiPxpAccelerator(Machine machine)
        {
            sysbus = machine.GetSystemBus(this);
            stopwatch = Stopwatch.StartNew();
            regs = new uint[32];
            try
            {
                if(File.Exists(LogPath))
                {
                    File.Delete(LogPath);
                }
            }
            catch
            {
            }
            Log("pxp accel ready csharp");
        }

        public uint ReadDoubleWord(long offset)
        {
            var index = offset / 4;
            if(index < 0 || index >= regs.Length)
            {
                return 0;
            }
            return regs[index];
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            var index = offset / 4;
            if(index >= 0 && index < regs.Length)
            {
                regs[index] = value;
            }

            if(offset == Status && value == StatusPending)
            {
                regs[StatusIndex] = StatusPending;
                regs[ResultIndex] = 0xFFFFFFFFu;
                var before = stopwatch.ElapsedMilliseconds;
                uint result;
                try
                {
                    result = RunRequest();
                }
                catch(Exception e)
                {
                    result = 0xFFFFFFFFu;
                    regs[ExceptionCountIndex]++;
                    Log("exception " + e.GetType().Name + ": " + e.Message);
                }

                var elapsed = (uint)Math.Max(0, stopwatch.ElapsedMilliseconds - before);
                regs[ResultIndex] = result;
                regs[ElapsedMsIndex] = elapsed;
                regs[TransferCountIndex]++;
                regs[LastBytesIndex] = lastBytesWritten;
                regs[TotalMsIndex] += elapsed;
                if(elapsed > regs[MaxMsIndex])
                {
                    regs[MaxMsIndex] = elapsed;
                }
                regs[StatusIndex] = result == 0 ? StatusDone : StatusError;

                var count = regs[TransferCountIndex];
                if(count == 1 || count % 25 == 0)
                {
                    Log(String.Format("transfer count={0} rc={1} elapsed_ms={2} src={3}x{4} dst={5}x{6} sfmt={7} dfmt={8}",
                        count, result, elapsed, regs[SrcWIndex], regs[SrcHIndex],
                        regs[DstWIndex], regs[DstHIndex], regs[SrcFmtIndex], regs[DstFmtIndex]));
                }
            }
        }

        public void Reset()
        {
            Array.Clear(regs, 0, regs.Length);
        }

        public long Size => 0x100;

        private uint RunRequest()
        {
            if(regs[CommandIndex] != CommandTransform)
            {
                return 0xFFFFFFF9u;
            }
            return RunTransform();
        }

        private uint RunTransform()
        {
            var srcPtr = regs[SrcIndex];
            var dstPtr = regs[DstIndex];
            var srcW = (int)regs[SrcWIndex];
            var srcH = (int)regs[SrcHIndex];
            var dstW = (int)regs[DstWIndex];
            var dstH = (int)regs[DstHIndex];
            var srcFmt = regs[SrcFmtIndex];
            var dstFmt = regs[DstFmtIndex];

            lastBytesWritten = 0;
            if(srcPtr == 0 || dstPtr == 0)
            {
                return 0xFFFFFFFEu;
            }
            if(srcW <= 0 || srcH <= 0 || dstW <= 0 || dstH <= 0)
            {
                return 0xFFFFFFFDu;
            }
            if(dstW > srcW || dstH > srcH)
            {
                return 0xFFFFFFFCu;
            }
            if((srcFmt != FmtXrgb8888 && srcFmt != FmtRgb888) ||
               (dstFmt != FmtRgb888 && dstFmt != FmtY8))
            {
                return 0xFFFFFFFBu;
            }
            var srcBpp = srcFmt == FmtRgb888 ? 3 : 4;
            var dstBpp = dstFmt == FmtY8 ? 1 : 3;
            var srcBytes = (long)srcW * srcH * srcBpp;
            var dstBytes = (long)dstW * dstH * dstBpp;
            if(srcBytes <= 0 || dstBytes <= 0 ||
               srcBytes > MaxTransferBytes || dstBytes > MaxTransferBytes)
            {
                return 0xFFFFFFFAu;
            }

            var input = sysbus.ReadBytes((ulong)srcPtr, (int)srcBytes);
            var output = new byte[(int)dstBytes];

            for(var oy = 0; oy < dstH; ++oy)
            {
                var y0 = (oy * srcH) / dstH;
                var y1 = ((oy + 1) * srcH) / dstH;
                if(y1 <= y0)
                {
                    y1 = y0 + 1;
                }

                for(var ox = 0; ox < dstW; ++ox)
                {
                    var x0 = (ox * srcW) / dstW;
                    var x1 = ((ox + 1) * srcW) / dstW;
                    if(x1 <= x0)
                    {
                        x1 = x0 + 1;
                    }

                    var count = (x1 - x0) * (y1 - y0);
                    uint sumR = 0;
                    uint sumG = 0;
                    uint sumB = 0;
                    uint sumY = 0;

                    for(var sy = y0; sy < y1; ++sy)
                    {
                        var baseOffset = (sy * srcW + x0) * srcBpp;
                        for(var sx = x0; sx < x1; ++sx)
                        {
                            byte r;
                            byte g;
                            byte b;
                            if(srcFmt == FmtRgb888)
                            {
                                r = input[baseOffset + 0];
                                g = input[baseOffset + 1];
                                b = input[baseOffset + 2];
                            }
                            else
                            {
                                b = input[baseOffset + 0];
                                g = input[baseOffset + 1];
                                r = input[baseOffset + 2];
                            }
                            sumR += r;
                            sumG += g;
                            sumB += b;
                            sumY += (77u * r + 150u * g + 29u * b) >> 8;
                            baseOffset += srcBpp;
                        }
                    }

                    var half = (uint)count / 2u;
                    var rAvg = (byte)((sumR + half) / (uint)count);
                    var gAvg = (byte)((sumG + half) / (uint)count);
                    var bAvg = (byte)((sumB + half) / (uint)count);

                    if(dstFmt == FmtY8)
                    {
                        output[oy * dstW + ox] = (byte)((sumY + half) / (uint)count);
                    }
                    else
                    {
                        var dstOffset = (oy * dstW + ox) * 3;
                        output[dstOffset + 0] = rAvg;
                        output[dstOffset + 1] = gAvg;
                        output[dstOffset + 2] = bAvg;
                    }
                }
            }

            sysbus.WriteBytes(output, (ulong)dstPtr);
            lastBytesWritten = (uint)output.Length;
            return 0;
        }

        private void Log(string message)
        {
            try
            {
                File.AppendAllText(LogPath, String.Format("t={0}ms {1}{2}",
                    stopwatch.ElapsedMilliseconds, message, Environment.NewLine));
            }
            catch
            {
            }
        }

        private readonly IBusController sysbus;
        private readonly Stopwatch stopwatch;
        private readonly uint[] regs;
        private uint lastBytesWritten;

        private const string LogPath = "/tmp/sentai_emu_pxp_accel.log";
        private const long Status = 0x00;

        private const int StatusIndex = 0;
        private const int CommandIndex = 1;
        private const int SrcIndex = 2;
        private const int SrcWIndex = 3;
        private const int SrcHIndex = 4;
        private const int SrcFmtIndex = 5;
        private const int DstIndex = 6;
        private const int DstWIndex = 7;
        private const int DstHIndex = 8;
        private const int DstFmtIndex = 9;
        private const int ResultIndex = 10;
        private const int SeqIndex = 11;
        private const int ElapsedMsIndex = 12;
        private const int TransferCountIndex = 13;
        private const int LastBytesIndex = 14;
        private const int TotalMsIndex = 15;
        private const int MaxMsIndex = 16;
        private const int ExceptionCountIndex = 17;

        private const uint StatusPending = 1;
        private const uint StatusDone = 2;
        private const uint StatusError = 3;
        private const uint CommandTransform = 1;
        private const uint FmtXrgb8888 = 0;
        private const uint FmtRgb888 = 1;
        private const uint FmtY8 = 2;
        private const int MaxTransferBytes = 4 * 1024 * 1024;
    }
}
