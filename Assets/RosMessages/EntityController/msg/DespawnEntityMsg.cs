//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.EntityController
{
    [Serializable]
    public class DespawnEntityMsg : Message
    {
        public const string k_RosMessageName = "entity_controller_msgs/DespawnEntity";
        public override string RosMessageName => k_RosMessageName;

        public string unique_id;

        public DespawnEntityMsg()
        {
            this.unique_id = "";
        }

        public DespawnEntityMsg(string unique_id)
        {
            this.unique_id = unique_id;
        }

        public static DespawnEntityMsg Deserialize(MessageDeserializer deserializer) => new DespawnEntityMsg(deserializer);

        private DespawnEntityMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.unique_id);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.unique_id);
        }

        public override string ToString()
        {
            return "DespawnEntityMsg: " +
            "\nunique_id: " + unique_id.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
