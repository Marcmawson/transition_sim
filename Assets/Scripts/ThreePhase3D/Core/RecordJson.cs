using System;
using System.IO;
using System.Runtime.Serialization.Json;
using System.Security.Cryptography;
using System.Text;

namespace PhysicsSim.ThreePhase3D.Core
{
    public static class RecordJson
    {
        public static byte[] Encode<T>(T value)
        {
            using (var stream = new MemoryStream())
            { Serializer<T>().WriteObject(stream, value); return stream.ToArray(); }
        }
        public static T Decode<T>(byte[] value)
        {
            using (var stream = new MemoryStream(value))
                return (T)Serializer<T>().ReadObject(stream);
        }
        static DataContractJsonSerializer Serializer<T>() => new DataContractJsonSerializer(typeof(T),
            new DataContractJsonSerializerSettings { UseSimpleDictionaryFormat = true });
        public static string Hash(byte[] bytes)
        {
            using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(bytes)).Replace("-", "").ToLowerInvariant();
        }
        public static string FileHash(string path) => Hash(File.ReadAllBytes(path));
        public static string Utf8Hash(string text) => Hash(Encoding.UTF8.GetBytes(text));
    }
}
