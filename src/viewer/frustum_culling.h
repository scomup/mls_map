#ifndef SAMPLE_CARTO_TOP_VIEWER_FRUSTUM_CULLING_H_
#define SAMPLE_CARTO_TOP_VIEWER_FRUSTUM_CULLING_H_

#include <pangolin/pangolin.h>
#include <tuple>

class FrustumCulling
{

  public:
    FrustumCulling()
    {
        float p[16];   // projection matrix
        float mv[16];  // model-view matrix
        float mvp[16]; // model-view-projection matrix
        float t;

        glGetFloatv(GL_PROJECTION_MATRIX, p);
        glGetFloatv(GL_MODELVIEW_MATRIX, mv);

        //
        // Concatenate the projection matrix and the model-view matrix to produce
        // a combined model-view-projection matrix.
        //

        mvp[0] = mv[0] * p[0] + mv[1] * p[4] + mv[2] * p[8] + mv[3] * p[12];
        mvp[1] = mv[0] * p[1] + mv[1] * p[5] + mv[2] * p[9] + mv[3] * p[13];
        mvp[2] = mv[0] * p[2] + mv[1] * p[6] + mv[2] * p[10] + mv[3] * p[14];
        mvp[3] = mv[0] * p[3] + mv[1] * p[7] + mv[2] * p[11] + mv[3] * p[15];

        mvp[4] = mv[4] * p[0] + mv[5] * p[4] + mv[6] * p[8] + mv[7] * p[12];
        mvp[5] = mv[4] * p[1] + mv[5] * p[5] + mv[6] * p[9] + mv[7] * p[13];
        mvp[6] = mv[4] * p[2] + mv[5] * p[6] + mv[6] * p[10] + mv[7] * p[14];
        mvp[7] = mv[4] * p[3] + mv[5] * p[7] + mv[6] * p[11] + mv[7] * p[15];

        mvp[8] = mv[8] * p[0] + mv[9] * p[4] + mv[10] * p[8] + mv[11] * p[12];
        mvp[9] = mv[8] * p[1] + mv[9] * p[5] + mv[10] * p[9] + mv[11] * p[13];
        mvp[10] = mv[8] * p[2] + mv[9] * p[6] + mv[10] * p[10] + mv[11] * p[14];
        mvp[11] = mv[8] * p[3] + mv[9] * p[7] + mv[10] * p[11] + mv[11] * p[15];

        mvp[12] = mv[12] * p[0] + mv[13] * p[4] + mv[14] * p[8] + mv[15] * p[12];
        mvp[13] = mv[12] * p[1] + mv[13] * p[5] + mv[14] * p[9] + mv[15] * p[13];
        mvp[14] = mv[12] * p[2] + mv[13] * p[6] + mv[14] * p[10] + mv[15] * p[14];
        mvp[15] = mv[12] * p[3] + mv[13] * p[7] + mv[14] * p[11] + mv[15] * p[15];

        //
        // Extract the frustum's right clipping plane and normalize it.
        //

        frustumPlanes_[0][0] = mvp[3] - mvp[0];
        frustumPlanes_[0][1] = mvp[7] - mvp[4];
        frustumPlanes_[0][2] = mvp[11] - mvp[8];
        frustumPlanes_[0][3] = mvp[15] - mvp[12];

        t = (float)sqrt(frustumPlanes_[0][0] * frustumPlanes_[0][0] +
                        frustumPlanes_[0][1] * frustumPlanes_[0][1] +
                        frustumPlanes_[0][2] * frustumPlanes_[0][2]);

        frustumPlanes_[0][0] /= t;
        frustumPlanes_[0][1] /= t;
        frustumPlanes_[0][2] /= t;
        frustumPlanes_[0][3] /= t;

        //
        // Extract the frustum's left clipping plane and normalize it.
        //

        frustumPlanes_[1][0] = mvp[3] + mvp[0];
        frustumPlanes_[1][1] = mvp[7] + mvp[4];
        frustumPlanes_[1][2] = mvp[11] + mvp[8];
        frustumPlanes_[1][3] = mvp[15] + mvp[12];

        t = (float)sqrt(frustumPlanes_[1][0] * frustumPlanes_[1][0] +
                        frustumPlanes_[1][1] * frustumPlanes_[1][1] +
                        frustumPlanes_[1][2] * frustumPlanes_[1][2]);

        frustumPlanes_[1][0] /= t;
        frustumPlanes_[1][1] /= t;
        frustumPlanes_[1][2] /= t;
        frustumPlanes_[1][3] /= t;

        //
        // Extract the frustum's bottom clipping plane and normalize it.
        //

        frustumPlanes_[2][0] = mvp[3] + mvp[1];
        frustumPlanes_[2][1] = mvp[7] + mvp[5];
        frustumPlanes_[2][2] = mvp[11] + mvp[9];
        frustumPlanes_[2][3] = mvp[15] + mvp[13];

        t = (float)sqrt(frustumPlanes_[2][0] * frustumPlanes_[2][0] +
                        frustumPlanes_[2][1] * frustumPlanes_[2][1] +
                        frustumPlanes_[2][2] * frustumPlanes_[2][2]);

        frustumPlanes_[2][0] /= t;
        frustumPlanes_[2][1] /= t;
        frustumPlanes_[2][2] /= t;
        frustumPlanes_[2][3] /= t;

        //
        // Extract the frustum's top clipping plane and normalize it.
        //

        frustumPlanes_[3][0] = mvp[3] - mvp[1];
        frustumPlanes_[3][1] = mvp[7] - mvp[5];
        frustumPlanes_[3][2] = mvp[11] - mvp[9];
        frustumPlanes_[3][3] = mvp[15] - mvp[13];

        t = (float)sqrt(frustumPlanes_[3][0] * frustumPlanes_[3][0] +
                        frustumPlanes_[3][1] * frustumPlanes_[3][1] +
                        frustumPlanes_[3][2] * frustumPlanes_[3][2]);

        frustumPlanes_[3][0] /= t;
        frustumPlanes_[3][1] /= t;
        frustumPlanes_[3][2] /= t;
        frustumPlanes_[3][3] /= t;

        //
        // Extract the frustum's far clipping plane and normalize it.
        //

        frustumPlanes_[4][0] = mvp[3] - mvp[2];
        frustumPlanes_[4][1] = mvp[7] - mvp[6];
        frustumPlanes_[4][2] = mvp[11] - mvp[10];
        frustumPlanes_[4][3] = mvp[15] - mvp[14];

        t = (float)sqrt(frustumPlanes_[4][0] * frustumPlanes_[4][0] +
                        frustumPlanes_[4][1] * frustumPlanes_[4][1] +
                        frustumPlanes_[4][2] * frustumPlanes_[4][2]);

        frustumPlanes_[4][0] /= t;
        frustumPlanes_[4][1] /= t;
        frustumPlanes_[4][2] /= t;
        frustumPlanes_[4][3] /= t;

        //
        // Extract the frustum's near clipping plane and normalize it.
        //

        frustumPlanes_[5][0] = mvp[3] + mvp[2];
        frustumPlanes_[5][1] = mvp[7] + mvp[6];
        frustumPlanes_[5][2] = mvp[11] + mvp[10];
        frustumPlanes_[5][3] = mvp[15] + mvp[14];

        t = (float)sqrt(frustumPlanes_[5][0] * frustumPlanes_[5][0] +
                        frustumPlanes_[5][1] * frustumPlanes_[5][1] +
                        frustumPlanes_[5][2] * frustumPlanes_[5][2]);

        frustumPlanes_[5][0] /= t;
        frustumPlanes_[5][1] /= t;
        frustumPlanes_[5][2] /= t;
        frustumPlanes_[5][3] /= t;
    }

    bool isBoundingSphereInFrustum(float x, float y, float z)
    {
        for (int i = 0; i < 6; ++i)
        {
            if (frustumPlanes_[i][0] * x +
                    frustumPlanes_[i][1] * y +
                    frustumPlanes_[i][2] * z +
                    frustumPlanes_[i][3] <=
                0)
                return false;
        }

        return true;
    }

  private:
    float frustumPlanes_[6][4];
};

#endif