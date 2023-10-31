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
        /// Topic name for transform.
        /// </summary>
        public string tfTopic = "/tf_static";

        /// <summary>
        /// Transform frame id.
        /// </summary>
        public string frameId = "_link";

        /// <summary>
        /// Parent frame id.
        /// </summary>
        public string parentFrameId = "_link";

        [SerializeField] bool isStatic;

        /// <summary>
        /// QoS settings.
        /// </summary>
        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1,
        };

        float timer = 0;
        int publishHz = 10;

        IPublisher<tf2_msgs.msg.TFMessage> tfPublisher;
        tf2_msgs.msg.TFMessage tfMsg;
        // tf2_ros.TransformBroadcaster br;
        geometry_msgs.msg.TransformStamped tfTrans;
        Transform t_transform;


        // Start is called before the first frame update
        void Start()
        {
            tfMsg = new tf2_msgs.msg.TFMessage()
            {
                Transforms = new geometry_msgs.msg.TransformStamped[1],
            };

            tfTrans = new geometry_msgs.msg.TransformStamped()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = parentFrameId,
                },
                Child_frame_id = frameId,
            }; 

            // tfMsg.Transforms = new geometry_msgs.msg.TransformStamped[1];
            // tfMsg.Transforms[0].Header.Frame_id = parentFrameId;
            // tfMsg.Transforms[0].Child_frame_id = frameId;

            // tfTrans.Header.Frame_id = parentFrameId;
            // tfTrans.Child_frame_id = frameId;

            // br.sendTransform(tfTrans);

            // Create publisher
            var qos = qosSettings.GetQoSProfile();
            tfPublisher = SimulatorROS2Node.CreatePublisher<tf2_msgs.msg.TFMessage>(tfTopic, qos);

            // t_transform = GetComponent<Transform>();
            t_transform = transform;

            if (isStatic == true)
            {
                var rosLocalPosition = ROS2Utility.UnityToRosPosition(t_transform.localPosition);
                var rosLocalRotation = ROS2Utility.UnityToRosRotation(t_transform.localRotation);

                // tfMsg.Transforms.Transform[0].Translation.X = rosLocalPosition.x;
                // tfMsg.Transforms.Transform[0].Translation.Y = rosLocalPosition.y;
                // tfMsg.Transforms.Transform[0].Translation.Z = rosLocalPosition.z;
                // tfMsg.Transforms.Transform[0].Rotation.X = rosLocalRotation.x;
                // tfMsg.Transforms.Transform[0].Rotation.Y = rosLocalRotation.y;
                // tfMsg.Transforms.Transform[0].Rotation.Z = rosLocalRotation.z;


                tfTrans.Transform.Translation.X = rosLocalPosition.x;
                tfTrans.Transform.Translation.Y = rosLocalPosition.y;
                tfTrans.Transform.Translation.Z = rosLocalPosition.z;
                tfTrans.Transform.Rotation.X = rosLocalRotation.x;
                tfTrans.Transform.Rotation.Y = rosLocalRotation.y;
                tfTrans.Transform.Rotation.Z = rosLocalRotation.z;
                tfTrans.Transform.Rotation.W = rosLocalRotation.w;

                // Update Stamp
                var tfTransHeader = tfTrans as MessageWithHeader;
                SimulatorROS2Node.UpdateROSTimestamp(ref tfTransHeader);

                tfMsg.Transforms[0] = tfTrans;

                tfPublisher.Publish(tfMsg);


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
                    var rosLocalPosition = ROS2Utility.UnityToRosPosition(t_transform.localPosition);
                    var rosLocalRotation = ROS2Utility.UnityToRosRotation(t_transform.localRotation);

                    // tfMsg.Transforms.Transform[0].Translation.X = rosLocalPosition.x;
                    // tfMsg.Transforms.Transform[0].Translation.Y = rosLocalPosition.y;
                    // tfMsg.Transforms.Transform[0].Translation.Z = rosLocalPosition.z;
                    // tfMsg.Transforms.Transform[0].Rotation.X = rosLocalRotation.x;
                    // tfMsg.Transforms.Transform[0].Rotation.Y = rosLocalRotation.y;
                    // tfMsg.Transforms.Transform[0].Rotation.Z = rosLocalRotation.z;


                    tfTrans.Transform.Translation.X = rosLocalPosition.x;
                    tfTrans.Transform.Translation.Y = rosLocalPosition.y;
                    tfTrans.Transform.Translation.Z = rosLocalPosition.z;
                    tfTrans.Transform.Rotation.X = rosLocalRotation.x;
                    tfTrans.Transform.Rotation.Y = rosLocalRotation.y;
                    tfTrans.Transform.Rotation.Z = rosLocalRotation.z;
                    tfTrans.Transform.Rotation.W = rosLocalRotation.w;

                    // Update Stamp
                    var tfTransHeader = tfTrans as MessageWithHeader;
                    SimulatorROS2Node.UpdateROSTimestamp(ref tfTransHeader);

                    tfMsg.Transforms[0] = tfTrans;

                    tfPublisher.Publish(tfMsg);
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
