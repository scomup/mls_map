/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "viewer.h"
#include <pangolin/pangolin.h>
#include "myhandler.h"

#include "Eigen/Core"
#include "Eigen/Geometry"

#include <iostream>
#include <chrono>
#include <thread>

#include "draw_object_helper.h"

Viewer::Viewer()
{
    t_ = 1e3 / 30;
    image_width_ = 640;
    image_height_ = 480;
    view_point_x_ = 20;
    view_point_y_ = 20;
    view_point_z_ = 10;
    view_point_f_ = 2000;
}

void Viewer::Run()
{
    finish_ = false;

    pangolin::CreateWindowAndBind("rrt", 1024, 768);

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuOriginalMap("menu.show original map", false, true);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, view_point_f_, view_point_f_, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(view_point_x_, view_point_y_, view_point_z_, 0, 0, 0, 0.1, 0.0, 1.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    auto handler = new pangolin::MyHandler3D(s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                                .SetHandler(handler);

    //glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);

    GLfloat lightColor0[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat lightPos0[] = {0.0f, 0.0f, 100.0f, 1.0f};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor0);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    while (!pangolin::ShouldQuit() && !finish_)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        d_cam.Activate(s_cam);
        pangolin::glDrawAxis(1);


        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_QUADS);
        //const GridMap<SPListST> &mls = *this;
        Vector2ui num_cell = mls_->getNumCells();
        for (size_t x = 0; x < num_cell.x(); x++)
        {
            for (size_t y = 0; y < num_cell.y(); y++)
            {
                auto &list = mls_->at(x, y);

                for (auto it = list.begin(); it != list.end(); it++)
                {
                    double top = it->getTop();
                    double height = top - it->getBottom();

                    //if(it->isVertical())
                    //glColorHSV(0);
                    //else
                    glColorHSV(std::abs(top * 100));
                    //printf("%f\n", top);

                    Vector3d pos(0, 0, 0);
                    mls_->fromGrid(maps::grid::Index(x, y), pos);
                    //DrawBox2(pos.x(), pos.y(), top, 0.01, 0.01);
                    DrawBox2(pos.x(), pos.y(), top, height, 0.05);

                    //pangolin::glDrawLine(xx, yy, zz, xx2, yy2, zz2);
                    //cout <<"Mean:"<<it->getMean() <<std::endl;
                    //cout <<"Variance:"<<it->getVariance() <<std::endl;
                    //cout <<"Height:"<<it->getHeight() <<std::endl;
                    //std::cout<<"------------------------\n";
                    //std::cout <<"Min:"<<it->getMin() <<std::endl;
                    //std::cout <<"Max:"<<it->getMax() <<std::endl;
                    //std::cout <<"getNormal:"<<it->getNormal() <<std::endl;
                }
                // auto graph = graph_->graph()[0];
            }
        }
        glEnd();
        

        for (auto &m : graph_->graph())
        {
            const auto &edge = m.second;
            
            for (auto &e : edge)
            {
                uint v = e.second;
                uint u = m.first;
                auto uu = graph_->positions()[u];
                auto vv = graph_->positions()[v];
                pangolin::glColorHSV(std::abs((uu.z() + vv.z()) * 50));
                glBegin(GL_LINES);
                glVertex3f(uu.x(), uu.y(), uu.z());
                glVertex3f(vv.x(), vv.y(), vv.z());
                glEnd();
            }
        }


        pangolin::FinishFrame();
    }
    SetFinish();
}

void Viewer::SetFinish()
{
    finish_ = true;
}
