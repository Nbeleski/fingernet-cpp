using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Runtime.InteropServices;
using System.Runtime.Versioning;
using System.Text;
using System.Threading.Tasks;

namespace antheus
{
    internal class FingerNet
    {
        const string dllname = @"fingernet.dll";

        [SupportedOSPlatform("windows")]
        static private Bitmap GerarBitmap(byte[] pixels, int w, int h)
        {
            Bitmap bmp = new Bitmap(w, h, PixelFormat.Format8bppIndexed);
            BitmapData data = bmp.LockBits(new Rectangle(0, 0, bmp.Width, bmp.Height), ImageLockMode.ReadWrite, bmp.PixelFormat);
            byte[] buffer = new byte[data.Stride * h];
            for (int i = 0; i < h; i++)
                for (int j = 0; j < w; j++)
                    buffer[i * data.Stride + j] = pixels[i * w + j];
            System.Runtime.InteropServices.Marshal.Copy(buffer, 0, data.Scan0, buffer.Length);
            bmp.UnlockBits(data);

            ColorPalette pal = bmp.Palette;
            for (int i = 0; i < 256; i++)
                pal.Entries[i] = Color.FromArgb(255, i, i, i);
            bmp.Palette = pal;
            return bmp;
        }

        [SupportedOSPlatform("windows")]
        static private byte[] PegarPixels(Bitmap bmp)
        {
            BitmapData data = bmp.LockBits(new Rectangle(0, 0, bmp.Width, bmp.Height), ImageLockMode.ReadWrite, bmp.PixelFormat);
            int stride = data.Stride;
            byte[] buffer;
            byte[] pixels;

            try
            {
                if (bmp.PixelFormat == PixelFormat.Format8bppIndexed)
                {
                    buffer = new byte[stride * bmp.Height];
                    pixels = new byte[bmp.Width * bmp.Height];
                    System.Runtime.InteropServices.Marshal.Copy(data.Scan0, buffer, 0, stride * bmp.Height);
                    for (int i = 0; i < bmp.Height; i++)
                        Array.Copy(buffer, i * stride, pixels, i * bmp.Width, bmp.Width);
                }
                else if (bmp.PixelFormat == PixelFormat.Format24bppRgb)
                {
                    buffer = new byte[stride * bmp.Height * 3];
                    pixels = new byte[bmp.Width * bmp.Height * 3];
                    System.Runtime.InteropServices.Marshal.Copy(data.Scan0, buffer, 0, stride * bmp.Height);

                    for (int i = 0; i < bmp.Height; i++)
                    {
                        for (int j = 0; j < bmp.Width; j++)
                        {
                            // RGB ou BGR????
                            pixels[3 * i * bmp.Width + 3 * j + 2] = buffer[i * stride + 3 * j + 0];
                            pixels[3 * i * bmp.Width + 3 * j + 1] = buffer[i * stride + 3 * j + 1];
                            pixels[3 * i * bmp.Width + 3 * j + 0] = buffer[i * stride + 3 * j + 2];
                        }
                    }
                }
                else
                    throw new ApplicationException("Formato de bitmap nao suportado: " + bmp.PixelFormat.ToString());

            }
            finally
            {
                bmp.UnlockBits(data);
            }


            return pixels;
        }

        [DllImport(dllname, CallingConvention = CallingConvention.Cdecl)]
        static private extern int extract_fingerprint_features(byte[] raw, int w, int h, byte[] mask, byte[] enhanced, int[] minutiae, ref int num_minutiae);

        [SupportedOSPlatform("windows")]
        static public int extractFingerprintFeatures(String filename, Bitmap image)
        {
            var raw = PegarPixels(image);
            var mask = new byte[raw.Length];
            var enhanced = new byte[raw.Length];
            var minutiae = new int[255];
            var num_minutiae = 0;

            var sw = new Stopwatch();
            sw.Start();

            var ret = extract_fingerprint_features(raw, image.Width, image.Height, mask, enhanced, minutiae, ref num_minutiae);
            if (ret != 0) // != ReturnCode::Ok
            {
                Console.WriteLine(ret);
            }

            sw.Stop();
            Console.WriteLine(" - {0} minutiae found in {1}", num_minutiae, sw.Elapsed);

            var enhanced_img = GerarBitmap(enhanced, image.Width, image.Height);

            var radius = 5;
            Bitmap rgbImage = new Bitmap(image.Width, image.Height, PixelFormat.Format24bppRgb);
            using (Graphics graphics = Graphics.FromImage(rgbImage))
            {
                graphics.DrawImage(image, 0, 0);
            }


            using (Graphics graphics = Graphics.FromImage(rgbImage))
            {
                for (var i = 0; i < num_minutiae * 3; i += 3)
                {
                    int x = minutiae[i + 0];
                    int y = minutiae[i + 1];
                    int cx = x - radius;
                    int cy = y - radius;
                    int diameter = radius * 2;
                    graphics.DrawEllipse(new Pen(Brushes.Red, 2), cx, cy, diameter, diameter);

                    int endX = (int)(x + 3 * radius * Math.Cos(Math.PI * minutiae[i + 2] / 180.0));
                    int endY = (int)(y + 3 * radius * Math.Sin(Math.PI * minutiae[i + 2] / 180.0));

                    graphics.DrawLine(new Pen(Brushes.Red, 2), x, y, endX, endY);
                }
            }

            enhanced_img.Save(filename + "_enhanced.png");
            rgbImage.Save(filename + "_minutiae.png");

            return 0;
        }

    }
}
