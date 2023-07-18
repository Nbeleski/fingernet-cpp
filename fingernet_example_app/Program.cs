using System;
using System.Drawing;
using System.IO;
using System.Runtime.Versioning;

namespace FingernetExample
{
    class Program
    {
        [SupportedOSPlatform("windows")]
        static string GetFileName(string path)
        {
            // Split the path using the directory separator characters
            string[] pathParts = path.Split('\\', '/');
            if (pathParts.Length == 0 )
            {
                return path;
            }

            // Get the last part of the path, which represents the filename
            string filename = pathParts[pathParts.Length - 1];

            return filename;
        }

        [SupportedOSPlatform("windows")]
        static void Main(string[] args)
        {
            if (args.Length == 0)
            {
                Console.WriteLine("No file argument provided.");
                return;
            }

            string filePath = args[0];

            if (!File.Exists(filePath))
            {
                Console.WriteLine("File does not exist.");
                return;
            }

            // Call a function on the opened file
            var image = new Bitmap(filePath);
            foo.FingerNet.extractFingerprintFeatures(GetFileName(filePath), image);
        }
    }
}