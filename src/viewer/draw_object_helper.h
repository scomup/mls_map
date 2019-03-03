#ifndef SAMPLE_CARTO_TOP_VIEWER_DRAW_OBJECT_HELPER_H_
#define SAMPLE_CARTO_TOP_VIEWER_DRAW_OBJECT_HELPER_H_



#include <pangolin/pangolin.h>
#include <tuple>


inline void glColorHSV( GLfloat hue, GLfloat a=1.0f, GLfloat s=1.0f, GLfloat v=1.0f )
{
    const GLfloat h = hue / 60.0f;
    const int i = (int)floor(h);
    const GLfloat f = (i%2 == 0) ? 1-(h-i) : h-i;
    const GLfloat m = v * (1-s);
    const GLfloat n = v * (1-s*f);
    switch(i)
    {
    case 0: glColor4f(v,n,m,a); break;
    case 1: glColor4f(n,v,m,a); break;
    case 2: glColor4f(m,v,n,a); break;
    case 3: glColor4f(m,n,v,a); break;
    case 4: glColor4f(n,m,v,a); break;
    case 5: glColor4f(v,m,n,a); break;
    default:
        break;
    }
}

inline void DrawPoints(std::vector<Eigen::Matrix<float, 3, 1> >& points, double *twc)
{
    glPushMatrix();
    glMultMatrixd(twc);
    glColor3f(0.0f,1.0f,0.0f);
    glPointSize(2);  
    glBegin(GL_POINTS);
    for(auto p:points){
        glVertex3f(p.x(), p.y(), p.z());
    }
    glEnd();
    glPopMatrix();
}


inline void drawSphere(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat radius, GLfloat M=10, GLfloat N=10)
{
    float step_z = M_PI / M;
    float step_xy = 2 * M_PI / N;
    float x[4], y[4], z[4];

    float angle_z = 0.0;
    float angle_xy = 0.0;
    int i = 0, j = 0;
    glBegin(GL_QUADS);
    for (i = 0; i < M; i++)
    {
        angle_z = i * step_z;

        for (j = 0; j < N; j++)
        {
            angle_xy = j * step_xy;

            x[0] = radius * sin(angle_z) * cos(angle_xy);
            y[0] = radius * sin(angle_z) * sin(angle_xy);
            z[0] = radius * cos(angle_z);

            x[1] = radius * sin(angle_z + step_z) * cos(angle_xy);
            y[1] = radius * sin(angle_z + step_z) * sin(angle_xy);
            z[1] = radius * cos(angle_z + step_z);

            x[2] = radius * sin(angle_z + step_z) * cos(angle_xy + step_xy);
            y[2] = radius * sin(angle_z + step_z) * sin(angle_xy + step_xy);
            z[2] = radius * cos(angle_z + step_z);

            x[3] = radius * sin(angle_z) * cos(angle_xy + step_xy);
            y[3] = radius * sin(angle_z) * sin(angle_xy + step_xy);
            z[3] = radius * cos(angle_z);

            for (int k = 0; k < 4; k++)
            {
                glVertex3f(xx + x[k], yy + y[k], zz + z[k]);
            }
        }
    }
    glEnd();
}

inline void DrawBox(GLfloat r, GLfloat x, GLfloat y, GLfloat z)
{



    
    GLfloat l[3] = {x - r / 2, y - r / 2, z - r / 2};
    GLfloat h[3] = {x + r / 2, y + r / 2, z + r / 2};

    //Front
    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertex3f(h[0], l[1], l[2]);
    glVertex3f(h[0], l[1], h[2]);
    glVertex3f(l[0], l[1], h[2]);
    glVertex3f(l[0], l[1], l[2]);
    
    //Back
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(l[0], h[1], l[2]);
    glVertex3f(l[0], h[1], h[2]);
    glVertex3f(h[0], h[1], h[2]);
    glVertex3f(h[0], h[1], l[2]);

    //Top
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(h[0], l[1], h[2]);
    glVertex3f(h[0], h[1], h[2]);
    glVertex3f(l[0], h[1], h[2]);
    glVertex3f(l[0], l[1], h[2]);

    //Bottom
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(h[0], h[1], l[2]);
    glVertex3f(h[0], l[1], l[2]);
    glVertex3f(l[0], l[1], l[2]);
    glVertex3f(l[0], h[1], l[2]);
    
    //Left
    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(l[0], h[1], h[2]);
    glVertex3f(l[0], h[1], l[2]);
    glVertex3f(l[0], l[1], l[2]);
    glVertex3f(l[0], l[1], h[2]);

    //Right
    glNormal3f(1.0f, 0.0f, 0.0f);
    glVertex3f(h[0], l[1], h[2]);
    glVertex3f(h[0], l[1], l[2]);
    glVertex3f(h[0], h[1], l[2]);
    glVertex3f(h[0], h[1], h[2]);
    
    
}

inline void DrawBox2(GLfloat x, GLfloat y, GLfloat z, GLfloat hh =0.05, GLfloat r=0.05)
{   
    GLfloat l[3] = {x - r / 2, y - r / 2, z - hh};
    GLfloat h[3] = {x + r / 2, y + r / 2, z    };

    //Front
    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertex3f(h[0], l[1], l[2]);
    glVertex3f(h[0], l[1], h[2]);
    glVertex3f(l[0], l[1], h[2]);
    glVertex3f(l[0], l[1], l[2]);
    
    //Back
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(l[0], h[1], l[2]);
    glVertex3f(l[0], h[1], h[2]);
    glVertex3f(h[0], h[1], h[2]);
    glVertex3f(h[0], h[1], l[2]);

    //Top
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(h[0], l[1], h[2]);
    glVertex3f(h[0], h[1], h[2]);
    glVertex3f(l[0], h[1], h[2]);
    glVertex3f(l[0], l[1], h[2]);

    //Bottom
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(h[0], h[1], l[2]);
    glVertex3f(h[0], l[1], l[2]);
    glVertex3f(l[0], l[1], l[2]);
    glVertex3f(l[0], h[1], l[2]);
    
    //Left
    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(l[0], h[1], h[2]);
    glVertex3f(l[0], h[1], l[2]);
    glVertex3f(l[0], l[1], l[2]);
    glVertex3f(l[0], l[1], h[2]);

    //Right
    glNormal3f(1.0f, 0.0f, 0.0f);
    glVertex3f(h[0], l[1], h[2]);
    glVertex3f(h[0], l[1], l[2]);
    glVertex3f(h[0], h[1], l[2]);
    glVertex3f(h[0], h[1], h[2]);
}



GLint DrawPlane( GLdouble w, GLdouble d ,GLdouble h)
{
	GLdouble norm[3];
    
	glPushMatrix();
	glBegin( GL_POLYGON );
        glColor4f(0.5,0.5,0.5,0.5);
		norm[0]=1;norm[1]=0;norm[2]=0;
		glNormal3dv (norm);
		glVertex3d( w/2.0 ,  d/2.0 ,   h);
		glVertex3d( w/2.0 , -d/2.0 ,   h);
		glVertex3d(-w/2.0 , -d/2.0 ,   h);
		glVertex3d(-w/2.0 ,  d/2.0 ,   h);
	glEnd();
	glPopMatrix();

	return(0) ;
}

GLint DrawGrid(GLdouble w, GLdouble s)
{
    glLineWidth(1);
    glColor4f(0.0f,0.0f,0.0f,0.3f);
    for (double i = -w / 2; i <= w / 2; i+=s)
    {
        pangolin::glDrawLine(i, -w / 2, 0, i, w / 2, 0);
        pangolin::glDrawLine(-w / 2, i, 0, w / 2, i, 0);
    };
    //glColor4f(0.0f,0.0f,0.0f,0.2f);
    //pangolin::glDrawRect(-w/2,-w/2,w/2,w/2);
    //DrawPlane(w,w,-0.05);
    return (0);
}

#endif //SAMPLE_CARTO_TOP_VIEWER_DRAW_OBJECT_HELPER_H_
