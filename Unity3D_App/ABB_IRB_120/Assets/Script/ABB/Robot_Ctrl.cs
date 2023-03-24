/****************************************************************************
MIT License
Copyright(c) 2023 Roman Parak
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*****************************************************************************
Author   : Roman Parak
Email    : Roman.Parak @outlook.com
Github   : https://github.com/rparak
File Name: Robot_Ctrl.cs
****************************************************************************/

// System 
using System;
using System.Threading;
using System.IO;
using System.Net;
using System.Net.Sockets;
// Unity
using UnityEngine;
using Debug = UnityEngine.Debug;
// ABB EGM Lib.
using abb.egm;


public class Robot_Ctrl : MonoBehaviour
{
    public static class GlobalVariables_Main_Control
    {
        public static bool Connect, Disconnect;
        public static bool Is_Connected, Is_Disconnected;
        // Joint Space:
        //  Orientation {J1 .. J6} (°)
        public static float[] J_Orientation_Target = new float[6];
        //  Approximate time required to reach the target
        public static float J_Orientation_Smooth_time;
        //  In Position
        public static bool[] J_Orientation_In_Pos = new bool[6];
    }

    public static class ABB_EGM_Control
    {
        // IP Port Number and IP Address
        public static string ip_address;
        public static readonly int port_number = 6511;
        // Joint Space:
        //  Orientation {J1 .. J6} (°)
        public static float[] J_Orientation = new float[6];
        public static double[] J_Orientation_Read = new double[6];
        // Class thread information (is alive or not)
        public static bool is_alive = false;
        // Ready state
        public static bool ready = false;
    }

    // Class Control Robot {ABB Externally Guided Motion - EGM}
    private Egm_Control ABB_EGM_Control_Cls = new Egm_Control();

    // Control of the joints orientation
    //  Targets (n_targets x n_joints)
    private float[,] J_Orientation_Target = new float[7, 6];
    //  Index of the target
    private int target_id = 0;

    // Other variables
    private int main_state_egm_connection = 0;
    private int main_state_control_egm = 0;

    // Start is called before the first frame update
    void Start()
    {
        // Initialization of targets
        //  Index 0: {MoveAbsJ [[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]], ...}
        J_Orientation_Target[0, 0] = 0.0f;
        J_Orientation_Target[0, 1] = 0.0f;
        J_Orientation_Target[0, 2] = 0.0f;
        J_Orientation_Target[0, 3] = 0.0f;
        J_Orientation_Target[0, 4] = 0.0f;
        J_Orientation_Target[0, 5] = 0.0f;
        // Index 1: {MoveAbsJ [[110,50,-20,-30,-75,50]],[9E9,9E9,9E9,9E9,9E9,9E9]], ...}
        J_Orientation_Target[1, 0] = 110.0f;
        J_Orientation_Target[1, 1] = 50.0f;
        J_Orientation_Target[1, 2] = -20.0f;
        J_Orientation_Target[1, 3] = -30.0f;
        J_Orientation_Target[1, 4] = -75.0f;
        J_Orientation_Target[1, 5] = 50.0f;
        // Index 2: {MoveAbsJ [[0,-67,44,0,-67,0],[9E9,9E9,9E9,9E9,9E9,9E9]], ...}
        J_Orientation_Target[2, 0] = 0.0f;
        J_Orientation_Target[2, 1] = -67.0f;
        J_Orientation_Target[2, 2] = 44.0f;
        J_Orientation_Target[2, 3] = 0.0f;
        J_Orientation_Target[2, 4] = -67.0f;
        J_Orientation_Target[2, 5] = 0.0f;
        // Index 3: {MoveAbsJ [[90,0,0,0,90,0]],[9E9,9E9,9E9,9E9,9E9,9E9]], ...}
        J_Orientation_Target[3, 0] = 90.0f;
        J_Orientation_Target[3, 1] = 0.0f;
        J_Orientation_Target[3, 2] = 0.0f;
        J_Orientation_Target[3, 3] = 0.0f;
        J_Orientation_Target[3, 4] = 90.0f;
        J_Orientation_Target[3, 5] = 0.0f;
        // Index 4: {MoveAbsJ [[0,-90,0,90,90,-90],[9E9,9E9,9E9,9E9,9E9,9E9]], ...}
        J_Orientation_Target[4, 0] = 0.0f;
        J_Orientation_Target[4, 1] = -90.0f;
        J_Orientation_Target[4, 2] = 0.0f;
        J_Orientation_Target[4, 3] = 90.0f;
        J_Orientation_Target[4, 4] = 90.0f;
        J_Orientation_Target[4, 5] = -90.0f;
        // Index 5: {MoveAbsJ [[0,40,50,90,90,90],[9E9,9E9,9E9,9E9,9E9,9E9]], ...}
        J_Orientation_Target[5, 0] = 0.0f;
        J_Orientation_Target[5, 1] = 40.0f;
        J_Orientation_Target[5, 2] = 50.0f;
        J_Orientation_Target[5, 3] = 90.0f;
        J_Orientation_Target[5, 4] = 90.0f;
        J_Orientation_Target[5, 5] = 90.0f;
        // Index 6: {MoveAbsJ [[90,50,-50,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]], ...}
        J_Orientation_Target[6, 0] = 90.0f;
        J_Orientation_Target[6, 1] = 50.0f;
        J_Orientation_Target[6, 2] = -50.0f;
        J_Orientation_Target[6, 3] = 0.0f;
        J_Orientation_Target[6, 4] = 0.0f;
        J_Orientation_Target[6, 5] = 0.0f;

        // Set the approximate time required to reach the target
        GlobalVariables_Main_Control.J_Orientation_Smooth_time = 2.0f;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        switch (main_state_egm_connection)
        {
            case 0:
                {
                    /*
                     Description:
                        Wait State {Disconnect State}
                     */

                    GlobalVariables_Main_Control.Is_Connected = false;
                    GlobalVariables_Main_Control.Is_Disconnected = true;

                    ABB_EGM_Control.ready = false;

                    if (GlobalVariables_Main_Control.Connect == true)
                    {
                        // Start Stream {ABB Externally Guided Motion - EGM}
                        ABB_EGM_Control_Cls.Start();

                        // go to connect state
                        main_state_egm_connection = 1;
                    }
                }
                break;
            case 1:
                {
                    /*
                     Description:
                        Data Processing State {Connect State}
                     */

                    GlobalVariables_Main_Control.Is_Connected = true;
                    GlobalVariables_Main_Control.Is_Disconnected = false;

                    if (GlobalVariables_Main_Control.Disconnect == true)
                    {
                        // Stop threading block
                        //  {ABB Externally Guided Motion - EGM}
                        if (ABB_EGM_Control.is_alive == true)
                        {
                            ABB_EGM_Control_Cls.Stop();
                        }

                        if (ABB_EGM_Control.is_alive == false)
                        {
                            // go to initialization state {wait state -> disconnect state}
                            main_state_egm_connection = 0;
                        }
                    }
                }
                break;
        }

        switch (main_state_control_egm)
        {
            case 0:
                {
                    /*
                     Description:
                        Wait State {Disconnect State}
                     */

                    target_id = 0;

                    // Set the joint target data as real
                    GlobalVariables_Main_Control.J_Orientation_Target[0] = (float)ABB_EGM_Control.J_Orientation_Read[0];
                    GlobalVariables_Main_Control.J_Orientation_Target[1] = (float)ABB_EGM_Control.J_Orientation_Read[1];
                    GlobalVariables_Main_Control.J_Orientation_Target[2] = (float)ABB_EGM_Control.J_Orientation_Read[2];
                    GlobalVariables_Main_Control.J_Orientation_Target[3] = (float)ABB_EGM_Control.J_Orientation_Read[3];
                    GlobalVariables_Main_Control.J_Orientation_Target[4] = (float)ABB_EGM_Control.J_Orientation_Read[4];
                    GlobalVariables_Main_Control.J_Orientation_Target[5] = (float)ABB_EGM_Control.J_Orientation_Read[5];

                    if (ABB_EGM_Control.ready == true)
                    {
                        main_state_control_egm = 1;
                    }
                }
                break;

            case 1:
                {
                    /*
                     Description:
                        Setting the desired orientation
                     */

                    if (ABB_EGM_Control.ready == false)
                    {
                        main_state_control_egm = 0;
                    }
                    else
                    {
                        // Set the desired orientation that can be read from the trajectory {Trajectory = J_Orientation_Target[,]}.
                        GlobalVariables_Main_Control.J_Orientation_Target[0] = J_Orientation_Target[target_id, 0];
                        GlobalVariables_Main_Control.J_Orientation_Target[1] = J_Orientation_Target[target_id, 1];
                        GlobalVariables_Main_Control.J_Orientation_Target[2] = J_Orientation_Target[target_id, 2];
                        GlobalVariables_Main_Control.J_Orientation_Target[3] = J_Orientation_Target[target_id, 3];
                        GlobalVariables_Main_Control.J_Orientation_Target[4] = J_Orientation_Target[target_id, 4];
                        GlobalVariables_Main_Control.J_Orientation_Target[5] = J_Orientation_Target[target_id, 5];

                        // Wait 100 ms as identification that the robot is in a moving state
                        System.Threading.Thread.Sleep(100);
                        main_state_control_egm = 2;
                    }
                }
                break;

            case 2:
                {
                    /*
                     Description:
                        Check if the robot is in position 
                     */

                    bool in_pos_all = true;
                    foreach (bool J_Orient_In_Pos_i in GlobalVariables_Main_Control.J_Orientation_In_Pos)
                    {
                        if (J_Orient_In_Pos_i == false)
                        {
                            in_pos_all = false;
                            break;
                        }
                    }

                    if (in_pos_all == true)
                    {
                        main_state_control_egm = 3;
                    }
                }
                break;

            case 3:
                {
                    /*
                     Description:
                        Check the trajectory length. If the desired orientation is in the final state, repeat the trajectory.
                     */

                    if (J_Orientation_Target.GetLength(0) - 1 == target_id)
                    {
                        target_id = 0;
                    }
                    else
                    {
                        target_id++;
                    }

                    main_state_control_egm = 1;
                }
                break;
        }
    }

    void OnApplicationQuit()
    {
        try
        {
            // Destroy Control Robot {ABB Externally Guided Motion - EGM}
            ABB_EGM_Control_Cls.Destroy();

            Destroy(this);
        }
        catch (Exception e)
        {
            Debug.LogException(e);
        }
    }

    class Egm_Control
    {
        private Thread sensor_thread = null;
        private UdpClient udp_client = null;
        private bool exit_thread = false;
        private uint sequence_number = 0;
        public void Egm_Control_Thread()
        {
            // Create an udp server and listen on any address and the port
            // {ABB Robot Port is set from the RobotStudio ABB}
            udp_client = new UdpClient(ABB_EGM_Control.port_number);

            // IPAddress.Parse(ABB_EGM_Control.ip_address)
            // IPAddress.Any
            var end_point = new IPEndPoint(IPAddress.Parse(ABB_EGM_Control.ip_address), ABB_EGM_Control.port_number);

            while (exit_thread == false)
            {
                // Get the data from the robot
                var data = udp_client.Receive(ref end_point);

                if (data != null)
                {
                    ABB_EGM_Control.ready = true;

                    // Initialization ABB Robot {EGM READ data (position, rotation)}
                    EgmRobot robot_msg = EgmRobot.CreateBuilder().MergeFrom(data).Build();
                    // Read robot joint orientation
                    ABB_EGM_Control.J_Orientation_Read[0] = robot_msg.FeedBack.Joints.GetJoints(0);
                    ABB_EGM_Control.J_Orientation_Read[1] = robot_msg.FeedBack.Joints.GetJoints(1);
                    ABB_EGM_Control.J_Orientation_Read[2] = robot_msg.FeedBack.Joints.GetJoints(2);
                    ABB_EGM_Control.J_Orientation_Read[3] = robot_msg.FeedBack.Joints.GetJoints(3);
                    ABB_EGM_Control.J_Orientation_Read[4] = robot_msg.FeedBack.Joints.GetJoints(4);
                    ABB_EGM_Control.J_Orientation_Read[5] = robot_msg.FeedBack.Joints.GetJoints(5);

                    // Create a new EGM sensor message
                    EgmSensor.Builder egm_sensor = EgmSensor.CreateBuilder();

                    // Create a sensor message to send to the robot
                    EMG_Sensor_Message(egm_sensor);

                    using (MemoryStream memory_stream = new MemoryStream())
                    {
                        // Sensor Message
                        EgmSensor sensor_message = egm_sensor.BuildPartial();
                        sensor_message.WriteTo(memory_stream);

                        // Send message to the ABB ROBOT {UDP}
                        int bytes_sent = udp_client.Send(memory_stream.ToArray(), (int)memory_stream.Length, end_point);

                        // Check sent data
                        if (bytes_sent < 0)
                        {
                            Debug.Log("Error send to robot");
                        }
                    }
                }
            }

            // Dispose the instance and terminate the UDP connection
            udp_client.Close();
        }
        void EMG_Sensor_Message(EgmSensor.Builder egm_s)
        {
            // create a header
            EgmHeader.Builder egm_hdr = new EgmHeader.Builder();
            /*
             * SetTm: Timestamp in milliseconds (can be used for monitoring delays)
             * SetMtype: Sent by sensor, MSGTYPE_DATA if sent from robot controller
             */
            egm_hdr.SetSeqno(sequence_number++).SetTm((uint)DateTime.Now.Ticks)
                .SetMtype(EgmHeader.Types.MessageType.MSGTYPE_CORRECTION);

            egm_s.SetHeader(egm_hdr);

            // Create EGM Sensor Data
            EgmPlanned.Builder planned = new EgmPlanned.Builder();
            EgmJoints.Builder joints = new EgmJoints.Builder();

            // Set data {Joints Orientation}
            //  Orientation {J1 .. J6} (°)
            joints.AddJoints(ABB_EGM_Control.J_Orientation[0]);
            joints.AddJoints(ABB_EGM_Control.J_Orientation[1]);
            joints.AddJoints(ABB_EGM_Control.J_Orientation[2]);
            joints.AddJoints(ABB_EGM_Control.J_Orientation[3]);
            joints.AddJoints(ABB_EGM_Control.J_Orientation[4]);
            joints.AddJoints(ABB_EGM_Control.J_Orientation[5]);

            // Bind position object to planned
            planned.SetJoints(joints);
            // Bind planned to sensor object
            egm_s.SetPlanned(planned);
        }
        public void Start()
        {
            exit_thread = false;
            // Start a thread and listen to incoming messages
            sensor_thread = new Thread(new ThreadStart(Egm_Control_Thread));
            sensor_thread.IsBackground = true;
            sensor_thread.Start();
            // Thread is active
            ABB_EGM_Control.is_alive = true;
        }
        public void Stop()
        {
            // Dispose the instance and terminate the TCP/IP connection
            udp_client.Close();

            // Stop and exit thread
            exit_thread = true;
            sensor_thread.Abort();
            Thread.Sleep(100);
            ABB_EGM_Control.is_alive = sensor_thread.IsAlive;
        }
        public void Destroy()
        {
            // Stop a thread (Robot Web Services communication)
            Stop();
            Thread.Sleep(100);
        }
    }
}
