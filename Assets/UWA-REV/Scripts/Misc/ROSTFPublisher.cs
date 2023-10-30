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

        IPublisher<tf2_msgs.msg.TFMessage> tfPublisher;
        tf2_msgs.msg.TFMessage tfMsg;
        Transform t_transform;


        // Start is called before the first frame update
        void Start()
        {
            tfMsg = new tf2_msgs.msg.TFMessage()
            {
            
                // Transforms.Add(geometry_msgs.msg.TransformStamped),
                Transforms = new geometry_msgs.msg.TransformStamped[40],
            };

            // tfMsg.Transforms.Add(new geometry_msgs.msg.TransformStamped());
            tfMsg.Transforms[0].Header.Frame_id = parentFrameId;
            tfMsg.Transforms[0].Child_frame_id = frameId;

            // Create publisher
            var qos = qosSettings.GetQoSProfile();
            tfPublisher = SimulatorROS2Node.CreatePublisher<tf2_msgs.msg.TFMessage>(tfTopic, qos);

            // t_transform = GetComponent<Transform>();
            t_transform = transform;

            if (isStatic == true)
            {
                var rosLocalPosition = ROS2Utility.UnityToRosPosition(t_transform.localPosition);
                var rosLocalRotation = ROS2Utility.UnityToRosRotation(t_transform.localRotation);

                tfMsg.Transforms[0].Transform.Translation.X = rosLocalPosition.x;
                tfMsg.Transforms[0].Transform.Translation.Y = rosLocalPosition.y;
                tfMsg.Transforms[0].Transform.Translation.Z = rosLocalPosition.z;
                tfMsg.Transforms[0].Transform.Rotation.X = rosLocalRotation.x;
                tfMsg.Transforms[0].Transform.Rotation.Y = rosLocalRotation.y;
                tfMsg.Transforms[0].Transform.Rotation.Z = rosLocalRotation.z;

            }
        }

        // Update is called once per frame
        void Update()
        {
            if (isStatic == false)
            {
                
            }
        }

        void OnDestroy()
        {

        }
    }
}
