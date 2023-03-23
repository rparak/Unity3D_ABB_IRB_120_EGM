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
File Name: UI_Ctrl.cs
****************************************************************************/

// System 
using System;
using System.Text;
// Unity
using UnityEngine;
using Debug = UnityEngine.Debug;
using UnityEngine.UI;
// TM 
using TMPro;

using static Robot_Ctrl;

public class UI_Ctrl : MonoBehaviour
{
    // TMP_InputField 
    public TMP_InputField ip_address_txt;
    // Image
    public Image connection_info_img;
    // TextMeshProUGUI
    public TextMeshProUGUI orient_j1_txt, orient_j2_txt, orient_j3_txt;
    public TextMeshProUGUI orient_j4_txt, orient_j5_txt, orient_j6_txt;
    public TextMeshProUGUI connectionInfo_txt;
    // Toggle
    public Toggle Viewpoint_Visibility;
    // GameObject
    public GameObject Viewpoint_EE_0;
    public GameObject Viewpoint_EE_SMC;
    public GameObject EE_SMC;
    public GameObject EE_SMC_Transparent;
    // Dropdown
    public TMP_Dropdown EE_Configuration;

    // Start is called before the first frame update
    void Start()
    {
        // Connection information {image} -> Connect/Disconnect
        connection_info_img.GetComponent<Image>().color = new Color32(255, 0, 48, 50);
        // Connection information {text} -> Connect/Disconnect
        connectionInfo_txt.text = "Disconnected";

        // Joint Orientation -> J1 .. J6 [degrees]
        orient_j1_txt.text = "0.00";
        orient_j2_txt.text = "0.00";
        orient_j3_txt.text = "0.00";
        orient_j4_txt.text = "0.00";
        orient_j5_txt.text = "0.00";
        orient_j6_txt.text = "0.00";

        // Robot IP Address
        ip_address_txt.text = "127.0.0.1";

        // Visibility of the robot's end effector viewpoint
        Viewpoint_Visibility.isOn = false;

        // End-Effector Configuration:
        //  0 - without end-effector
        //  1 - with predefined end-effector
        EE_Configuration.value = 0;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        // Read Variables:
        //  IP Address of the robot
        ABB_EGM_Control.ip_address = ip_address_txt.text;
        ABB_EGM_Control.ip_address = ip_address_txt.text;

        // Connection Information
        //  If the button (connect/disconnect) is pressed, change the color and text
        if (GlobalVariables_Main_Control.Is_Connected == true)
        {
            // green color
            connection_info_img.GetComponent<Image>().color = new Color32(135, 255, 0, 50);
            connectionInfo_txt.text = "Connected";
        }
        else if (GlobalVariables_Main_Control.Is_Disconnected == true)
        {
            // red color
            connection_info_img.GetComponent<Image>().color = new Color32(255, 0, 48, 50);
            connectionInfo_txt.text = "Disconnected";
        }

        // Cyclic read-write parameters to text info
        // Joint Orientation -> J1 .. J6 [degrees]
        orient_j1_txt.text = Math.Round(ABB_EGM_Control.J_Orientation[0], 2).ToString();
        orient_j2_txt.text = Math.Round(ABB_EGM_Control.J_Orientation[1], 2).ToString();
        orient_j3_txt.text = Math.Round(ABB_EGM_Control.J_Orientation[2], 2).ToString();
        orient_j4_txt.text = Math.Round(ABB_EGM_Control.J_Orientation[3], 2).ToString();
        orient_j5_txt.text = Math.Round(ABB_EGM_Control.J_Orientation[4], 2).ToString();
        orient_j6_txt.text = Math.Round(ABB_EGM_Control.J_Orientation[5], 2).ToString();


        // Configuration of the robot end-effector visualization in the scene
        switch (EE_Configuration.value)
        {
            case 0:
                {
                    // End-Effector Configuration: 
                    //  0 - without end-effector
                    EE_SMC.SetActive(false);
                    EE_SMC_Transparent.SetActive(false);

                    if (Viewpoint_Visibility.isOn == true)
                        Viewpoint_EE_0.SetActive(true);
                    else
                        Viewpoint_EE_0.SetActive(false);

                    Viewpoint_EE_SMC.SetActive(false);
                }
                break;

            case 1:
                {
                    // End-Effector Configuration:
                    //  1 - with predefined end-effector
                    EE_SMC.SetActive(true);
                    EE_SMC_Transparent.SetActive(true);

                    if (Viewpoint_Visibility.isOn == true)
                        Viewpoint_EE_SMC.SetActive(true);
                    else
                        Viewpoint_EE_SMC.SetActive(false);

                    Viewpoint_EE_0.SetActive(false);
                }
                break;
        }
    }

    void OnApplicationQuit()
    {
        try
        {
            Destroy(this);
        }
        catch (Exception e)
        {
            Debug.LogException(e);
        }
    }

    public void TaskOnClick_ConnectBTN()
    {
        GlobalVariables_Main_Control.Connect = true;
        GlobalVariables_Main_Control.Disconnect = false;
    }

    public void TaskOnClick_DisconnectBTN()
    {
        GlobalVariables_Main_Control.Connect = false;
        GlobalVariables_Main_Control.Disconnect = true;
    }
}
