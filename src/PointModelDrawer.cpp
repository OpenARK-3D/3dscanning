//
// Created by yang on 16-12-3.
//

#include "PointModelDrawer.h"
#include <pangolin/pangolin.h>

using namespace pcl;
using namespace std;
using namespace pangolin;

namespace ORB_SLAM2 {
    PointModelDrawer::PointModelDrawer() {
    }

// use by the point model thread
    void PointModelDrawer::update(PointCloud<PointXYZRGB>::Ptr model) {
        unique_lock<mutex> lock(mCloudMutex);
        if (mCloud.size() < model->size()) {
            mCloud.resize(model->size());
        }
        std::copy(model->begin(), model->end(), mCloud.begin());
    }

// use by the viewer thread
    void PointModelDrawer::drawModel() {
        static PointCloud<PointXYZRGB> cloud;
        static GLuint bufferObject = 0;
        static int bufferSize = 0;

        int count;
        {
            unique_lock<mutex> lock(mCloudMutex);
            count = mCloud.size();
            if (cloud.size() < count) {
                cloud.resize(count);
            }

            std::copy(mCloud.begin(), mCloud.end(), cloud.begin());

            mCloud.clear();
        }


        // create the buffer
        if (bufferObject == 0) {
            glGenBuffers(1, &bufferObject);
        }

        glBindBuffer(GL_ARRAY_BUFFER, bufferObject);
        int newsize = count * sizeof(PointXYZRGB);
        if (newsize > bufferSize) {
            // create a bigger size
            glBufferData(GL_ARRAY_BUFFER, newsize, cloud.points.data(), GL_DYNAMIC_DRAW);
            bufferSize = newsize;
        } else if (newsize > 0) {
            glBufferSubData(GL_ARRAY_BUFFER, 0, newsize, cloud.points.data());
        }

        glVertexPointer(3, GL_FLOAT, 32, 0);
        glColorPointer(3, GL_UNSIGNED_BYTE, 32, (void *) 16);

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        glPointSize(2.0);
        glDrawArrays(GL_POINTS, 0, cloud.size());

        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
}