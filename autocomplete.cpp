//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2021, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar

    \author    <pkunjam1@jhu.edu>
    \author    Punit Kunjam
*/
//==============================================================================

#include "autocomplete.h"
#include <boost/program_options.hpp>
#include <ambf_server/RosComBase.h>
#include <std_msgs/Bool.h>

using namespace std;

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
afAutoCompletePlugin::afAutoCompletePlugin()
{
    cout << "Constructer!!" << endl;
}
int afAutoCompletePlugin::init(int argc, char **argv, const afWorldPtr a_afWorld)
{   
    cout << "Init Function!!" << endl;

    m_rosNode = afROSNode::getNode();
    m_commLossSub = m_rosNode->subscribe<std_msgs::Bool>("/communication_loss", 1, &afAutoCompletePlugin::communication_loss_cb, this);

    m_psm1_recoverySub = m_rosNode->subscribe<std_msgs::Bool>("/psm1/recovery", 1 , &afAutoCompletePlugin::psm1_recovery_cb, this);
    m_psm2_recoverySub = m_rosNode->subscribe<std_msgs::Bool>("/psm2/recovery", 1 , &afAutoCompletePlugin::psm2_recovery_cb, this);

    m_worldPtr = a_afWorld;

    // Get first camera
    // m_mainCamera = m_worldPtr->getCamera("cameraL");
    // if (m_mainCamera){
    //     cerr << "INFO! GOT CAMERA: " << m_mainCamera->getName() << endl;
    // }
    // else{
    //     cerr << "WARNING! COULD NOT FIND main_camera" << endl;
    //     m_mainCamera = m_worldPtr->getCameras()[0];
    // }

    // Get camera
    m_stereoCameraL = m_worldPtr->getCamera("cameraL");
    m_stereoCameraR = m_worldPtr->getCamera("cameraR");

    m_PSM1Tool = m_worldPtr->getRigidBody("/ambf/env/psm1/BODY tool roll link");
    m_PSM2Tool = m_worldPtr->getRigidBody("/ambf/env/psm2/BODY tool roll link");
    m_PSM1pitch = m_worldPtr->getRigidBody("/ambf/env/psm1/BODY tool pitch link");
    m_PSM2pitch = m_worldPtr->getRigidBody("/ambf/env/psm2/BODY tool pitch link");
    m_PSM1yaw = m_worldPtr->getRigidBody("/ambf/env/psm1/BODY tool yaw link");
    m_PSM2yaw = m_worldPtr->getRigidBody("/ambf/env/psm2/BODY tool yaw link");
    m_PSM1gripper1 = m_worldPtr->getRigidBody("/ambf/env/psm1/BODY tool gripper1 link");
    m_PSM2gripper1 = m_worldPtr->getRigidBody("/ambf/env/psm2/BODY tool gripper1 link");
    m_PSM1gripper2 = m_worldPtr->getRigidBody("/ambf/env/psm1/BODY tool gripper2 link");
    m_PSM2gripper2 = m_worldPtr->getRigidBody("/ambf/env/psm2/BODY tool gripper2 link");

    m_PSM1_ghost_Tool = m_worldPtr->getRigidBody("/ambf/env/psm1_ghost/BODY tool roll link");
    m_PSM2_ghost_Tool = m_worldPtr->getRigidBody("/ambf/env/psm2_ghost/BODY tool roll link");
    m_PSM1_ghost_pitch = m_worldPtr->getRigidBody("/ambf/env/psm1_ghost/BODY tool pitch link");
    m_PSM2_ghost_pitch = m_worldPtr->getRigidBody("/ambf/env/psm2_ghost/BODY tool pitch link");
    m_PSM1_ghost_yaw = m_worldPtr->getRigidBody("/ambf/env/psm1_ghost/BODY tool yaw link");
    m_PSM2_ghost_yaw = m_worldPtr->getRigidBody("/ambf/env/psm2_ghost/BODY tool yaw link");    
    m_PSM1_ghost_gripper1 = m_worldPtr->getRigidBody("/ambf/env/psm1_ghost/BODY tool gripper1 link");
    m_PSM2_ghost_gripper1 = m_worldPtr->getRigidBody("/ambf/env/psm2_ghost/BODY tool gripper1 link");
    m_PSM1_ghost_gripper2 = m_worldPtr->getRigidBody("/ambf/env/psm1_ghost/BODY tool gripper2 link");
    m_PSM2_ghost_gripper2 = m_worldPtr->getRigidBody("/ambf/env/psm2_ghost/BODY tool gripper2 link");

    m_PSM1_remote_Tool = m_worldPtr->getRigidBody("/ambf/env/psm1_remote/BODY tool roll link");
    m_PSM2_remote_Tool = m_worldPtr->getRigidBody("/ambf/env/psm2_remote/BODY tool roll link");
    m_PSM1_remote_pitch = m_worldPtr->getRigidBody("/ambf/env/psm1_remote/BODY tool pitch link");
    m_PSM2_remote_pitch = m_worldPtr->getRigidBody("/ambf/env/psm2_remote/BODY tool pitch link");
    m_PSM1_remote_yaw = m_worldPtr->getRigidBody("/ambf/env/psm1_remote/BODY tool yaw link");
    m_PSM2_remote_yaw = m_worldPtr->getRigidBody("/ambf/env/psm2_remote/BODY tool yaw link");    
    m_PSM1_remote_gripper1 = m_worldPtr->getRigidBody("/ambf/env/psm1_remote/BODY tool gripper1 link");
    m_PSM2_remote_gripper1 = m_worldPtr->getRigidBody("/ambf/env/psm2_remote/BODY tool gripper1 link");
    m_PSM1_remote_gripper2 = m_worldPtr->getRigidBody("/ambf/env/psm1_remote/BODY tool gripper2 link");
    m_PSM2_remote_gripper2 = m_worldPtr->getRigidBody("/ambf/env/psm2_remote/BODY tool gripper2 link");


    m_peg1 = m_worldPtr->getRigidBody("/ambf/icl/BODY PuzzleRed1");
    m_peg2 = m_worldPtr->getRigidBody("/ambf/icl/BODY PuzzleRed2");
    m_peg3 = m_worldPtr->getRigidBody("/ambf/icl/BODY PuzzleRed3");
    m_peg4 = m_worldPtr->getRigidBody("/ambf/icl/BODY PuzzleRed4");
    m_peg5 = m_worldPtr->getRigidBody("/ambf/icl/BODY PuzzleRed5");
    m_peg6 = m_worldPtr->getRigidBody("/ambf/icl/BODY PuzzleYellow");

    m_shadow1 = m_worldPtr->getRigidBody("/ambf/icl/BODY One_shadow");
    m_shadow2 = m_worldPtr->getRigidBody("/ambf/icl/BODY Two_shadow");
    m_shadow3 = m_worldPtr->getRigidBody("/ambf/icl/BODY Three_shadow");
    m_shadow4 = m_worldPtr->getRigidBody("/ambf/icl/BODY Four_shadow");
    m_shadow5 = m_worldPtr->getRigidBody("/ambf/icl/BODY Five_shadow");
    m_shadow6 = m_worldPtr->getRigidBody("/ambf/icl/BODY Six_shadow");
    
    // m_shadow4->m_visualMesh->setShowEnabled(false);


    if(m_peg1) m_peg_flag = true;
    else m_peg_flag = false;

    m_tool_flag = true;
    m_ghost_flag = true;
    m_remote_flag = true;

    m_visualize_flag = false;
    m_control_flag = false;

    if(m_PSM1Tool)
    {
        cerr << "INFO! GOT TOOL:" << m_PSM1Tool->getName() << endl;
    }

    else{
        cerr << "WARNING! COULD NOT FIND Tool" << endl;
    }
    if(m_PSM2Tool)
    {
        cerr << "INFO! GOT TOOL:" << m_PSM2Tool->getName() << endl;
    }

    else{
        cerr << "WARNING! COULD NOT FIND Tool" << endl;
    }


    // create a font
    cFontPtr font = NEW_CFONTCALIBRI40();

    m_comStatus = new cLabel(font);
    m_comStatus->setLocalPos(m_stereoCameraL->m_width*0.4, m_stereoCameraL->m_height*0.85, 0);
    m_comStatus->m_fontColor.setRed();
    m_comStatus->setFontScale(0.8);
    m_comStatus->setText("Communication Lost");
    m_stereoCameraL->getFrontLayer()->addChild(m_comStatus);
    m_comStatus->setShowEnabled(false); 

    m_legend = new cLabel(font);
    m_legend->setLocalPos(m_stereoCameraL->m_width*0.7, m_stereoCameraL->m_height*0.85, 0);
    m_legend->m_fontColor.setBlue();
    m_legend->setFontScale(0.8);
    m_legend->setText("Blue: Remote-side robot");
    m_stereoCameraL->getFrontLayer()->addChild(m_legend);
    m_legend->setShowEnabled(false);

    m_ori_recovery = new cLabel(font);
    m_ori_recovery->setLocalPos(m_stereoCameraL->m_width*0.35, m_stereoCameraL->m_height*0.15, 0);
    m_ori_recovery->m_fontColor.setRed();
    m_ori_recovery->setFontScale(1.2);
    m_stereoCameraL->getFrontLayer()->addChild(m_ori_recovery);

    cBackground *background = new cBackground();
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.6f, 0.6f, 0.6f),
                                cColorf(0.6f, 0.6f, 0.6f));
    m_stereoCameraL->getBackLayer()->addChild(background);

    return 1;
}

void afAutoCompletePlugin::graphicsUpdate()
{
    m_comStatus->setShowEnabled(m_comloss_text); 
    m_legend->setShowEnabled(m_comloss_text);
    if (m_psm1_recovery){
        m_ori_recovery->setText("PSM1 Orientation Misaligned");
    }
    if (m_psm2_recovery){
        m_ori_recovery->setText("PSM2 Orientation Misaligned");
    }
    m_ori_recovery->setShowEnabled(m_psm1_recovery || m_psm2_recovery);
    m_comStatus->setLocalPos(m_stereoCameraL->m_width*0.4, m_stereoCameraL->m_height*0.85, 0);
    m_legend->setLocalPos(m_stereoCameraL->m_width*0.7, m_stereoCameraL->m_height*0.85, 0);
    m_ori_recovery->setLocalPos(m_stereoCameraL->m_width*0.35, m_stereoCameraL->m_height*0.15, 0);

    
    
    if(!m_comloss){

        if (m_peg_flag){


            m_T_1 = m_peg1->getLocalTransform();
            m_T_2 = m_peg2->getLocalTransform();
            m_T_3 = m_peg3->getLocalTransform();
            m_T_4 = m_peg4->getLocalTransform();
            m_T_5 = m_peg5->getLocalTransform();
            m_T_6 = m_peg6->getLocalTransform();

            V_i.set(0,0,0.098);
            m_T_1.setLocalPos(m_T_1.getLocalPos() +  m_T_1.getLocalRot() * V_i);
            m_T_2.setLocalPos(m_T_2.getLocalPos() +  m_T_2.getLocalRot() * V_i);
            m_T_3.setLocalPos(m_T_3.getLocalPos() +  m_T_3.getLocalRot() * V_i);
            m_T_4.setLocalPos(m_T_4.getLocalPos() +  m_T_4.getLocalRot() * V_i);
            m_T_5.setLocalPos(m_T_5.getLocalPos() +  m_T_5.getLocalRot() * V_i);
            m_T_6.setLocalPos(m_T_6.getLocalPos() +  m_T_6.getLocalRot() * V_i);
            m_shadow1->setLocalTransform(m_T_1);
            m_shadow2->setLocalTransform(m_T_2);
            m_shadow3->setLocalTransform(m_T_3);
            m_shadow4->setLocalTransform(m_T_4);
            m_shadow5->setLocalTransform(m_T_5);
            m_shadow6->setLocalTransform(m_T_6);
            // m_shadow2->setLocalTransform(m_peg2->getLocalTransform());
            // m_shadow3->setLocalTransform(m_peg3->getLocalTransform());
            // m_shadow4->setLocalTransform(m_peg4->getLocalTransform());
            // m_shadow5->setLocalTransform(m_peg5->getLocalTransform());
            // m_shadow6->setLocalTransform(m_peg6->getLocalTransform());
        }
        
    }


    if(!m_visualize_flag){

        if (!m_control_flag){
            m_PSM1Tool->m_visualMesh->setShowEnabled(m_comloss);
            m_PSM2Tool->m_visualMesh->setShowEnabled(m_comloss);
            m_PSM1pitch->m_visualMesh->setShowEnabled(m_comloss);
            m_PSM2pitch->m_visualMesh->setShowEnabled(m_comloss);
            m_PSM1yaw->m_visualMesh->setShowEnabled(m_comloss);
            m_PSM2yaw->m_visualMesh->setShowEnabled(m_comloss);
            m_PSM1gripper1->m_visualMesh->setShowEnabled(m_comloss);
            m_PSM2gripper1->m_visualMesh->setShowEnabled(m_comloss);
            m_PSM1gripper2->m_visualMesh->setShowEnabled(m_comloss);
            m_PSM2gripper2->m_visualMesh->setShowEnabled(m_comloss);

            // m_PSM1_remote_Tool->m_visualMesh->setColor(cColorf(0, 1, 1));
            // m_PSM2_remote_Tool->m_visualMesh->setVertexColor(cColorf(0, 1, 1));
            
        }


        // m_PSM1_ghost_Tool->m_visualMesh->setShowEnabled(m_comloss);
        // m_PSM2_ghost_Tool->m_visualMesh->setShowEnabled(m_comloss);
        // m_PSM1_ghost_pitch->m_visualMesh->setShowEnabled(m_comloss);
        // m_PSM2_ghost_pitch->m_visualMesh->setShowEnabled(m_comloss);
        m_PSM1_ghost_yaw->m_visualMesh->setShowEnabled((m_comloss || m_psm1_recovery));
        m_PSM2_ghost_yaw->m_visualMesh->setShowEnabled((m_comloss || m_psm2_recovery));
        m_PSM1_ghost_gripper1->m_visualMesh->setShowEnabled((m_comloss || m_psm1_recovery));
        m_PSM2_ghost_gripper1->m_visualMesh->setShowEnabled((m_comloss || m_psm2_recovery));
        m_PSM1_ghost_gripper2->m_visualMesh->setShowEnabled((m_comloss || m_psm1_recovery));
        m_PSM2_ghost_gripper2->m_visualMesh->setShowEnabled((m_comloss || m_psm2_recovery));

        if(m_control_flag){
            m_PSM1Tool->m_visualMesh->setShowEnabled(true);
            m_PSM2Tool->m_visualMesh->setShowEnabled(true);
            m_PSM1pitch->m_visualMesh->setShowEnabled(true);
            m_PSM2pitch->m_visualMesh->setShowEnabled(true);
            m_PSM1yaw->m_visualMesh->setShowEnabled(true);
            m_PSM2yaw->m_visualMesh->setShowEnabled(true);
            m_PSM1gripper1->m_visualMesh->setShowEnabled(true);
            m_PSM2gripper1->m_visualMesh->setShowEnabled(true);
            m_PSM1gripper2->m_visualMesh->setShowEnabled(true);
            m_PSM2gripper2->m_visualMesh->setShowEnabled(true);
            m_PSM1_ghost_yaw->m_visualMesh->setShowEnabled(false);
            m_PSM2_ghost_yaw->m_visualMesh->setShowEnabled(false);
            m_PSM1_ghost_gripper1->m_visualMesh->setShowEnabled(false);
            m_PSM2_ghost_gripper1->m_visualMesh->setShowEnabled(false);
            m_PSM1_ghost_gripper2->m_visualMesh->setShowEnabled(false);
            m_PSM2_ghost_gripper2->m_visualMesh->setShowEnabled(false);
        }
    }
}
void afAutoCompletePlugin::communication_loss_cb(const std_msgs::Bool::ConstPtr& comloss)
{
    m_comloss_text = comloss->data;
    // if(m_comloss_text ==false && m_comloss==true)
    //     sleep(1.0);
    m_comloss = m_comloss_text;
}

void afAutoCompletePlugin::psm1_recovery_cb(const std_msgs::Bool::ConstPtr& recovery)
{
    m_psm1_recovery = recovery->data;
}

void afAutoCompletePlugin::psm2_recovery_cb(const std_msgs::Bool::ConstPtr& recovery)
{
    m_psm2_recovery = recovery->data;
}


void afAutoCompletePlugin::physicsUpdate(double dt)
{       
        
        // if(m_comloss == false)   
        // {
        //     m_comStatus->setShowEnabled(m_comloss);  
        // }
        
}

void afAutoCompletePlugin::keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    if (a_mods == GLFW_MOD_CONTROL){
        if (a_key == GLFW_KEY_1){
            if (m_stereoCameraL){
                m_stereoCameraL->setLocalPos(m_stereoCameraL->getLocalPos() - cVector3d(0.001, 0., 0.));
            }
            if (m_stereoCameraR){
                m_stereoCameraR->setLocalPos(m_stereoCameraR->getLocalPos() + cVector3d(0.001, 0., 0.));
            }
        }
        else if (a_key == GLFW_KEY_2){
            if (m_stereoCameraL){
                m_stereoCameraL->setLocalPos(m_stereoCameraL->getLocalPos() + cVector3d(0.001, 0., 0.));
            }
            if (m_stereoCameraR){
                m_stereoCameraR->setLocalPos(m_stereoCameraR->getLocalPos() - cVector3d(0.001, 0., 0.));
            }
        }

        else if (a_key == GLFW_KEY_3){
            cout << "Toggling the tool visibility" << endl;

            m_PSM1Tool->m_visualMesh->setShowEnabled(!m_tool_flag);
            m_PSM2Tool->m_visualMesh->setShowEnabled(!m_tool_flag);
            m_PSM1pitch->m_visualMesh->setShowEnabled(!m_tool_flag);
            m_PSM2pitch->m_visualMesh->setShowEnabled(!m_tool_flag);
            m_PSM1yaw->m_visualMesh->setShowEnabled(!m_tool_flag);
            m_PSM2yaw->m_visualMesh->setShowEnabled(!m_tool_flag);
            m_PSM1gripper1->m_visualMesh->setShowEnabled(!m_tool_flag);
            m_PSM2gripper1->m_visualMesh->setShowEnabled(!m_tool_flag);
            m_PSM1gripper2->m_visualMesh->setShowEnabled(!m_tool_flag);
            m_PSM2gripper2->m_visualMesh->setShowEnabled(!m_tool_flag);

            m_tool_flag = !m_tool_flag;
        }

        else if (a_key == GLFW_KEY_4){
            cout << "Toggling the ghost tool visibility" << endl;
            
            m_PSM1_ghost_yaw->m_visualMesh->setShowEnabled(!m_ghost_flag);
            m_PSM2_ghost_yaw->m_visualMesh->setShowEnabled(!m_ghost_flag);
            m_PSM1_ghost_gripper1->m_visualMesh->setShowEnabled(!m_ghost_flag);
            m_PSM2_ghost_gripper1->m_visualMesh->setShowEnabled(!m_ghost_flag);
            m_PSM1_ghost_gripper2->m_visualMesh->setShowEnabled(!m_ghost_flag);
            m_PSM2_ghost_gripper2->m_visualMesh->setShowEnabled(!m_ghost_flag);

            m_ghost_flag = !m_ghost_flag;

        }

        else if (a_key == GLFW_KEY_5){
            cout << "Toggling the remote rool visibility" << endl;
            
            m_PSM1_remote_Tool->m_visualMesh->setShowEnabled(!m_remote_flag);
            m_PSM2_remote_Tool->m_visualMesh->setShowEnabled(!m_remote_flag);
            m_PSM1_remote_pitch->m_visualMesh->setShowEnabled(!m_remote_flag);
            m_PSM2_remote_pitch->m_visualMesh->setShowEnabled(!m_remote_flag);
            m_PSM1_remote_yaw->m_visualMesh->setShowEnabled(!m_remote_flag);
            m_PSM2_remote_yaw->m_visualMesh->setShowEnabled(!m_remote_flag);
            m_PSM1_remote_gripper1->m_visualMesh->setShowEnabled(!m_remote_flag);
            m_PSM2_remote_gripper1->m_visualMesh->setShowEnabled(!m_remote_flag);
            m_PSM1_remote_gripper2->m_visualMesh->setShowEnabled(!m_remote_flag);
            m_PSM2_remote_gripper2->m_visualMesh->setShowEnabled(!m_remote_flag);

            m_remote_flag = !m_remote_flag;

        }

        else if (a_key == GLFW_KEY_0){
            cout << "Changing the mode for control to:" << !m_control_flag << endl;
            
            m_control_flag = !m_control_flag;
        }

        else if (a_key == GLFW_KEY_F){
            cout << "Changing the visualization mode to:" << !m_visualize_flag << endl;
            
            m_visualize_flag = !m_visualize_flag;
        }
    }
}


