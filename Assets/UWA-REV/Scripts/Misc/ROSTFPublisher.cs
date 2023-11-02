using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    public class ROSTFPublisher : MonoBehaviour
    {
        [Header("ROS Topic parameters")]

        /// <summary>
        /// Transform frame id.
        /// </summary>
        public string frameId = "_link";

        /// <summary>
        /// Parent frame id.
        /// </summary>
        public string parentFrameId = "_link";

        /// <summary>
        /// Publish Speed.
        /// </summary>
        [SerializeField] int publishHz = 10;

        [SerializeField] bool isStatic;

        /// <summary>
        /// QoS settings.
        /// </summary>
        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 10,
        };

        float timer = 0;

        IPublisher<geometry_msgs.msg.TransformStamped> utfStaticPublisher;
        IPublisher<geometry_msgs.msg.TransformStamped> utfPublisher;
        geometry_msgs.msg.TransformStamped utfStaticMsg;
        geometry_msgs.msg.TransformStamped utfMsg;
        Transform t_transform;


        // Start is called before the first frame update
        void Start()
        {

            utfStaticMsg = new geometry_msgs.msg.TransformStamped()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = parentFrameId,
                },
                Child_frame_id = frameId,
            };

            utfMsg = new geometry_msgs.msg.TransformStamped()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = parentFrameId,
                },
                Child_frame_id = frameId,
            };

            // Create publisher
            var qos = qosSettings.GetQoSProfile();
            utfStaticPublisher = SimulatorROS2Node.CreatePublisher<geometry_msgs.msg.TransformStamped>("/utf_static", qos);
            utfPublisher = SimulatorROS2Node.CreatePublisher<geometry_msgs.msg.TransformStamped>("/utf", qos);

            t_transform = transform;

            if (isStatic == true)
            {
                var rosLocalPosition = ROS2Utility.UnityToRosPosition(t_transform.localPosition);
                var rosLocalRotation = ROS2Utility.UnityToRosRotation(t_transform.localRotation);

                utfStaticMsg.Transform.Translation.X = rosLocalPosition.x;
                utfStaticMsg.Transform.Translation.Y = rosLocalPosition.y;
                utfStaticMsg.Transform.Translation.Z = rosLocalPosition.z;
                utfStaticMsg.Transform.Rotation.X = rosLocalRotation.x;
                utfStaticMsg.Transform.Rotation.Y = rosLocalRotation.y;
                utfStaticMsg.Transform.Rotation.Z = rosLocalRotation.z;
                utfStaticMsg.Transform.Rotation.W = rosLocalRotation.w;

                // Update Stamp
                var utfStaticMsgHeader = utfStaticMsg as MessageWithHeader;
                SimulatorROS2Node.UpdateROSTimestamp(ref utfStaticMsgHeader);

                utfStaticPublisher.Publish(utfStaticMsg);
            }
        }

        bool NeedToPublish()
        {
            timer += Time.deltaTime;
            var interval = 1.0f / publishHz;
            interval -= 0.00001f;
            if (timer < interval)
                return false;
            timer = 0;
            return true;
        }

        void FixedUpdate()
        {
            // Provide publications with a given frequency
            if (NeedToPublish())
            {
                if (isStatic == false)
                {
                    t_transform = transform;

                    var rosLocalPosition = ROS2Utility.UnityToRosPosition(t_transform.localPosition);
                    var rosLocalRotation = ROS2Utility.UnityToRosRotation(t_transform.localRotation);


                    utfMsg.Transform.Translation.X = rosLocalPosition.x;
                    utfMsg.Transform.Translation.Y = rosLocalPosition.y;
                    utfMsg.Transform.Translation.Z = rosLocalPosition.z;
                    utfMsg.Transform.Rotation.X = rosLocalRotation.x;
                    utfMsg.Transform.Rotation.Y = rosLocalRotation.y;
                    utfMsg.Transform.Rotation.Z = rosLocalRotation.z;
                    utfMsg.Transform.Rotation.W = rosLocalRotation.w;

                    // Update Stamp
                    var utfMsgHeader = utfMsg as MessageWithHeader;
                    SimulatorROS2Node.UpdateROSTimestamp(ref utfMsgHeader);

                    utfPublisher.Publish(utfMsg);
                }
            }
        }

        // Update is called once per frame
        // void Update()
        // {
        //     if (isStatic == false)
        //     {
        //         var rosLocalPosition = ROS2Utility.UnityToRosPosition(t_transform.localPosition);
        //         var rosLocalRotation = ROS2Utility.UnityToRosRotation(t_transform.localRotation);

        //         // tfMsg.Transforms.Transform[0].Translation.X = rosLocalPosition.x;
        //         // tfMsg.Transforms.Transform[0].Translation.Y = rosLocalPosition.y;
        //         // tfMsg.Transforms.Transform[0].Translation.Z = rosLocalPosition.z;
        //         // tfMsg.Transforms.Transform[0].Rotation.X = rosLocalRotation.x;
        //         // tfMsg.Transforms.Transform[0].Rotation.Y = rosLocalRotation.y;
        //         // tfMsg.Transforms.Transform[0].Rotation.Z = rosLocalRotation.z;


        //         tfTrans.Transform.Translation.X = rosLocalPosition.x;
        //         tfTrans.Transform.Translation.Y = rosLocalPosition.y;
        //         tfTrans.Transform.Translation.Z = rosLocalPosition.z;
        //         tfTrans.Transform.Rotation.X = rosLocalRotation.x;
        //         tfTrans.Transform.Rotation.Y = rosLocalRotation.y;
        //         tfTrans.Transform.Rotation.Z = rosLocalRotation.z;
        //         tfTrans.Transform.Rotation.W = rosLocalRotation.w;

        //         // Update Stamp
        //         var tfTransHeader = tfTrans as MessageWithHeader;
        //         SimulatorROS2Node.UpdateROSTimestamp(ref tfTransHeader);

        //         tfMsg.Transforms[0] = tfTrans;

        //         tfPublisher.Publish(tfMsg);
        //     }
        // }

        void OnDestroy()
        {

        }
    }
}
