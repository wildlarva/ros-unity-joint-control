using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.Sensor;

public class RosJointControl : MonoBehaviour
{
    const int k_NumRobotJoints = 6;

    [SerializeField]
    string[] m_JointNames =
        { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };

    // Variables required for ROS communication
    [SerializeField]
    string m_JointStatesTopicName = "/sim_joint_states";
    [SerializeField]
    string m_JointCommandsTopicName = "/sim_joint_commands";

    [SerializeField]
    GameObject m_UrdfRobot;

    // Robot Joints
    struct Joint {
        public string jointName;
        public ArticulationBody articulationBody;
        public UrdfJointRevolute urdfJointRevolute;
    }

    Joint[] m_Joints;
    Dictionary<string, Joint> m_JointName2Joint;

    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        Debug.Log("RosJointControl: Start started.");
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<JointStateMsg>(m_JointStatesTopicName);

        m_JointName2Joint = new Dictionary<string, Joint>();

        var urdfJointRevoluteList = m_UrdfRobot.transform.GetComponentsInChildren<UrdfJointRevolute>();
        foreach (var urdfJointRevolute in urdfJointRevoluteList)
        {
            var joint = new Joint();
            joint.jointName = urdfJointRevolute.jointName;
            joint.urdfJointRevolute = urdfJointRevolute;
            joint.articulationBody = urdfJointRevolute.gameObject.GetComponent<ArticulationBody>();
            m_JointName2Joint.Add(urdfJointRevolute.jointName, joint);
        }

        m_Joints = new Joint[k_NumRobotJoints];
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            Joint joint;
            if (!m_JointName2Joint.TryGetValue(m_JointNames[i], out joint)) {
                continue;
            }

            m_Joints[i] = joint;
        }

        m_Ros.Subscribe<JointStateMsg>(m_JointCommandsTopicName, JointCommandsCallback);
    }

    void Update()
    {
        var jointStatesMessage = new JointStateMsg();
        jointStatesMessage.name = new string[k_NumRobotJoints];
        jointStatesMessage.position = new double[k_NumRobotJoints];

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            jointStatesMessage.name[i] = m_Joints[i].jointName;
            jointStatesMessage.position[i] = m_Joints[i].urdfJointRevolute.GetPosition();
        }

        m_Ros.Publish(m_JointStatesTopicName, jointStatesMessage);
    }

    private void JointCommandsCallback(JointStateMsg message) {
        Debug.Log("RosJointControl: JointCommandsCallback started.");
        Debug.Log("RosJointControl: JointCommandsCallback: received positions=" + message.position);

        if (message.position.Length < k_NumRobotJoints) {
            Debug.Log("RosJointControl: JointCommandsCallback: required 6 positions, but " + message.position.Length);
            return;
        }

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            var joint1XDrive = m_Joints[i].articulationBody.xDrive;
            joint1XDrive.target = ((float) message.position[i]) * Mathf.Rad2Deg;
            m_Joints[i].articulationBody.xDrive = joint1XDrive;
        }
    }
}
