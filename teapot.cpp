/* OS/X:  gcc teapot.cpp -framework OpenGL -framework GLUT */

#include <stdio.h>
#include <string.h>
#include "GLUT/glut.h"
#include <math.h>

typedef struct {
    float x, y, z, w;
} Quat;

typedef struct {
    float a, x, y, z;
} AxisAngle;

typedef struct {
    Quat q0, c0, c1, q1;
} Spline;


static double PI=3.1415926;
static int timeDelay=60;

/* Globe stuff */
float glat = 0.0f;
float glon = 0.0f;

/* Menu processing stuff */
int mousedown_x, mousedown_y;
int mousedown_line, mousedown_col;

/* Longitude/Latitude/Heading for the main points */
float lon[16], lat[16], hea[16];
unsigned int nPoints=0;

/* Quaternion representations of the main points */
Quat quat[16];

/* Splines between the points */
Spline spline[15];

/* Interpolation parameter */
float t = 0.0;

/* Tension/Continuity/Bias parameters for the splines */
float ptens = 0.0;
float pcont = 0.0;
float pbias = 0.0;


/* Stuff to do with printing strings on an OGL window using GLUT */

void outputString(float x, float y, char *string)
{
    int len, i;
    
    if (x!=0 || y!=0) {
        glRasterPos2f(x, y);
    }
        
    len = (int) strlen(string);
    for (i = 0; i < len; i++) {
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, string[i]);
    }
}

void outputQuat(float x, float y, Quat* q)
{
    char buffer[256];
    sprintf(buffer, "%+6.2f %+6.2f %+6.2f %+6.2f", q->w, q->x, q->y, q->z);
    outputString(x, y, buffer);
}

void outputFloat(float x, float y, float v)
{
    char buffer[16];
    sprintf(buffer, "%+6.2f", v);
    outputString(x, y, buffer);
}

void outputAngle(float x, float y, float v)
{
    outputFloat(x, y, v * 180 / PI);
}

void outputInteger(float x, float y, int v)
{
    char buffer[16];
    sprintf(buffer, "%02d", v);
    outputString(x, y, buffer);
}

/* Stuff to do with the mathematics of quaternions */

void conjugateQuat(Quat& o, Quat& q)
{
    o.w = +q.w;
    o.x = -q.x;
    o.y = -q.y;
    o.z = -q.z;
}

void addQuat(Quat& o, Quat& a, Quat& b)
{
    o.w = a.w + b.w;
    o.x = a.x + b.x;
    o.y = a.y + b.y;
    o.z = a.z + b.z;
}

void scaleQuat(Quat& o, Quat& q, float s)
{
    o.w = q.w * s;
    o.x = q.x * s;
    o.y = q.y * s;
    o.z = q.z * s;
}

void copyQuat(Quat& o, Quat& i)
{
    o.w = i.w;
    o.x = i.x;
    o.y = i.y;
    o.z = i.z;
}

void multiplyQuat(Quat& o, Quat& a, Quat& b)
{
    o.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    o.x = a.x*b.w + a.w*b.x + a.y*b.z - a.z*b.y;
    o.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    o.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
}

float dotQuat(Quat& a, Quat& b)
{
    return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

void slerpQuat(Quat& out, Quat& a, Quat& b, float t)
{
    float angle = acos(dotQuat(a, b));
    float sa,sb;
    if (angle > 0.001) {
        sa = sin((1-t)*angle)/sin(angle);
        sb = sin(t*angle)/sin(angle);
    } else {
        sa = 1-t;
        sb = t;
    }
        
    Quat q1, q2;
    scaleQuat(q1, a, sa);
    scaleQuat(q2, b, sb);
    addQuat(out, q1, q2);
}

void normalizeQuat(Quat& q)
{
    float d = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    q.w /= d;
    q.x /= d;
    q.y /= d;
    q.z /= d;
}

void expQuat(Quat& o, Quat& i)
{
    float ab = sqrt(i.x*i.x + i.y*i.y + i.z*i.z);

    float r = exp(i.w) * cos(ab);
    float t = exp(i.w) * sin(ab);
        
    float m = (ab == 0.0) ? 0 : (t / ab);

    o.w = r;
    o.x = m * i.x;
    o.y = m * i.y;
    o.z = m * i.z;
}

void logQuat(Quat& o, Quat& i)
{
    float ab = sqrt(i.x*i.x + i.y*i.y + i.z*i.z);

    float r = 0.5 * log(i.w * i.w + ab * ab );
    float t = atan2(ab, i.w);

    float m = (ab == 0.0) ? 0 : (t / ab);

    o.w = r;
    o.x = m * i.x;
    o.y = m * i.y;
    o.z = m * i.z;
}

/* Stuff to do with the mathematics of matrices */

void multVecMatrix(float*out, float* v, float* m)
{
    out[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
    out[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
    out[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

void oglMultMatrix(float* m)
{
    float m4[16];
    m4[0 ] = m[0];
    m4[1 ] = m[3];
    m4[2 ] = m[6];
    m4[3 ] = 0.0f;
    m4[4 ] = m[1];
    m4[5 ] = m[4];
    m4[6 ] = m[7];
    m4[7 ] = 0.0f;
    m4[8 ] = m[2];
    m4[9 ] = m[5];
    m4[10] = m[8];
    m4[11] = 0.0f;
    m4[12] = 0.0f;
    m4[13] = 0.0f;
    m4[14] = 0.0f;
    m4[15] = 1.0f;
    glMultMatrixf(m4);
}

/* Stuff to do with conversions between quaternions and other representations */

void quatToMatrix(Quat& q, float* mat)
{
    float x2 = q.x * q.x;
    float y2 = q.y * q.y;
    float z2 = q.z * q.z;
    mat[0] = 1 - 2 * y2 - 2 * z2;
    mat[1] = 2 * q.x * q.y - 2 * q.z * q.w;
    mat[2] = 2 * q.x * q.z + 2 * q.y * q.w;
    mat[3] = 2 * q.x * q.y + 2 * q.z * q.w;
    mat[4] = 1 - 2 * x2 - 2 * z2;
    mat[5] = 2 * q.y * q.z - 2 * q.x * q.w;
    mat[6] = 2 * q.x * q.z - 2 * q.y * q.w;
    mat[7] = 2 * q.y * q.z + 2 * q.x * q.w;
    mat[8] = 1 - 2 * x2 - 2 * y2;
}

void matrixToQuat(float* m, Quat* q)
{
    q->w = sqrt((1 + m[0] + m[4] + m[8]) / 2);
    q->x = (m[7] - m[5]) / (4 * q->w);
    q->y = (m[2] - m[6]) / (4 * q->w);
    q->z = (m[3] - m[1]) / (4 * q->w);
}

void quatToEuler(Quat& q, float& lon, float& lat, float& hea)
{
    lon = atan2(2 * q.y*q.w - 2 * q.x * q.z, 1 - 2 * q.y*q.y - 2 * q.z * q.z);
    lat = asin(2 * q.x * q.y + 2 * q.z * q.w);
    hea = atan2(2 * q.x * q.w - 2 * q.y * q.z, 1 - 2 * q.x * q.x - 2 * q.z * q.z);
}

void eulerToQuat(float lon, float lat, float hea, Quat& q)
{
    float clon = cos(lon / 2);
    float clat = cos(lat / 2);
    float chea = cos(hea / 2);
    float slon = sin(lon / 2);
    float slat = sin(lat / 2);
    float shea = sin(hea / 2);
        
    q.w = clon*clat*chea - slon*slat*shea;
    q.x = slon*slat*chea + clon*clat*shea;
    q.y = slon*clat*chea + clon*slat*shea;
    q.z = clon*slat*chea - slon*clat*shea;
        
    normalizeQuat(q);
}

void quatToAxisAngle(AxisAngle& aa, Quat& q)
{
    float qq = sqrt(1 - q.w * q.w);
    aa.a = 2 * acos(q.w);
    aa.x = q.x / qq;
    aa.y = q.y / qq;
    aa.z = q.z / qq;
}

void axisAngleToQuat(AxisAngle& aa, Quat& q)
{
    float sa = sin(aa.a / 2);
    float ca = cos(aa.a / 2);
    q.w = ca;
    q.x = aa.x * sa;
    q.y = aa.y * sa;
    q.z = aa.z * sa;
    normalizeQuat(q);
}

/* Stuff to do with interpolation of quaternions along a spline */

void interpolateSpline(Quat& out, Spline& spline, float p)
{
    float a = p;
    Quat q1, q2;
    slerpQuat(q1, spline.q0, spline.q1, a);
    slerpQuat(q2, spline.c0, spline.c1, a);
    slerpQuat(out, q1, q2, 4 * a * (1 - a));
}

void createSplines()
{
    if (nPoints > 1) {

        if (nPoints > 2) {

            Quat c[16];
            copyQuat(c[0], quat[0]);
            copyQuat(c[nPoints-1], quat[nPoints-1]);

            for (int i=1; i<nPoints-1; i++) {

                Quat inv;
                conjugateQuat(inv, quat[i]);
                Quat q0, q1, q2, q3;
                multiplyQuat(q0, quat[i+1], inv);
                multiplyQuat(q1, quat[i-1], inv);
                logQuat(q2, q0);
                logQuat(q3, q1);
                addQuat(q0, q2, q3);
                scaleQuat(q1, q0, -0.25 * (1-ptens));
                expQuat(q0, q1);
                multiplyQuat(c[i], q0, quat[i]);
            }

            float p1 = (pcont + pbias) / 2.0;
            float p2 = (pcont - pbias) / 2.0;

            copyQuat(spline[0].q0, quat[0]);
            copyQuat(spline[0].c0, c[0]);
            slerpQuat(spline[0].c0, c[0], c[1], p2);
            for (int i=1; i<nPoints-1; i++) {
                copyQuat(spline[i-1].q1, quat[i+0]);
                slerpQuat(spline[i-1].c1, c[i+0], c[i-1], p1);
                copyQuat(spline[i].q0, quat[i+0]);
                slerpQuat(spline[i].c0, c[i+0], c[i+1], p2);
            }
            copyQuat(spline[nPoints-2].q1, quat[nPoints-1]);
            slerpQuat(spline[nPoints-2].c1, c[nPoints-1], c[nPoints-2], p1);
        } else {
            copyQuat(spline[0].q0, quat[0]);
            copyQuat(spline[0].c0, quat[0]);
            copyQuat(spline[0].q1, quat[1]);
            copyQuat(spline[0].c1, quat[1]);
        }
    }
}

/* Stuff to do with drawing the OpenGL scene */

void init(void)
{
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glShadeModel(GL_SMOOTH);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glCullFace(GL_BACK);

    float color[4] = { 1.0f,0.5f,0.5f,1.0f };
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color);
    glMaterialf(GL_FRONT, GL_SHININESS, 0.5);

    float lightPos[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glEnable(GL_LIGHT0);
        
    nPoints = 1;
    lat[0] = 0;
    lon[0] = 0;
    hea[0] = 0;
    eulerToQuat(lon[0], lat[0], hea[0], quat[0]);
}

void drawTeapot()
{
    /* Interpolate teapot direction */
    Quat tQuat;
    if (nPoints > 1) {
        unsigned int nSteps = (nPoints-1);
        unsigned int n = (unsigned int)floor(t*nSteps);
        float param = (t * nSteps) - (n);
        interpolateSpline(tQuat, spline[n], param);
    } else {
        copyQuat(tQuat, quat[0]);
    }

    /* Prepare to draw the teapot */
    glEnable(GL_LIGHTING);
    GLUquadric* quad;
    glPushMatrix();

    /* Rotate the coordinate system around so that the spout faces longitude=0 */
    glRotatef(90, 0.0, 1.0, 0.0);

    /* Rotate the teapot to the correct facing */
    float m[9];
    quatToMatrix(tQuat, m);
    oglMultMatrix(m);

    /* Finally, angle the teapot down so that the spout points at the rotation point */
    glPushMatrix();
    glRotatef(-15, 0.0, 0.0, 1.0);

    /* Draw the teapot */
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glutSolidTeapot(0.5);
    glDisable(GL_LIGHTING);
    glPopMatrix();

    /* Draw the rotation axis of the teapot */
    AxisAngle aa;
    quatToAxisAngle(aa, tQuat);
    glColor3f(1.0, 0.0, 0.0);
    glLineWidth(3);
    glBegin(GL_LINES);
    glVertex3f(0.0,0.0,0.0);
    glVertex3f(aa.x, aa.y, aa.z);
    glEnd();

    glPopMatrix();
}

void drawGlobe()
{
    GLUquadric* quad;
    glPushMatrix();
    glRotatef(90, 1.0, 0.0, 0.0);
    quad = gluNewQuadric();
    glColor4f(1.0, 1.0, 1.0, 0.1);
    glLineWidth(1.0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    gluSphere(quad, 0.81, 32, 32);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    gluDeleteQuadric(quad);
    glPopMatrix();
}

void drawSpline()
{
    if (nPoints > 1) {
        float point[3] = { 0.9, 0.0, 0.0 };
        glPushMatrix();
        glRotatef(90, 0, 1, 0);
        glLineWidth(2);
        glPointSize(3);

        /* Draw the line that the teapot follows - the transformed spatial coordinate */
        glColor4f(1.0,0.0,0.0,1.0);
        glBegin(GL_LINE_STRIP);
        for (float t = 0.0; t <= 1.0; t += 0.01) {
            unsigned int nSteps = (nPoints-1);
            unsigned int n = (unsigned int)floor(t*nSteps);
            float param = (t * nSteps) - (n);

            Quat q;
            interpolateSpline(q, spline[n], param);

            float m[9];
            quatToMatrix(q, m);

            float v[3];
            multVecMatrix(v, point, m);

            glVertex3fv(v);
        }
        glEnd();

        /* Draw the points showing even steps of the interpolation parameter */
        glColor3f(0.0,1.0,0.0);
        glBegin(GL_POINTS);

        for (float t = 0.0; t <= 1.0; t += 0.01) {
            unsigned int nSteps = (nPoints-1);
            unsigned int n = (unsigned int)floor(t*nSteps);
            float param = (t * nSteps) - (n);

            Quat q;
            interpolateSpline(q, spline[n], param);

            float m[9];
            quatToMatrix(q, m);

            float v[3];
            multVecMatrix(v, point, m);

            glVertex3fv(v);
        }
        glEnd();

        /* Draw the line tracing the movements of the interpolated quaterion's axial component */
        glColor4f(1.0,0.0,1.0,1.0);
        glBegin(GL_LINE_STRIP);
        for (float t = 0.0; t <= 1.0; t += 0.01) {
            unsigned int nSteps = (nPoints-1);
            unsigned int n = (unsigned int)floor(t*nSteps);
            float param = (t * nSteps) - (n);
                        
            Quat q;
            interpolateSpline(q, spline[n], param);
                        
            AxisAngle aa;
            quatToAxisAngle(aa, q);

            glVertex3f(aa.x, aa.y, aa.z);
        }
        glEnd();
                
        glPopMatrix();
    }
}

void drawQuat(Quat& q)
{
    static float size=0.05;
    float c0[3] = { 0.8, 0.0, 0.0 };
    float c1[3] = { 1.0, 0.0, 0.0 };        
    float c2[3] = { 0.9, 0.0, 0.0 };
    float c3[3] = { 0.9, 0.1, 0.0 };        
    float m[9];
    float t0[3],t1[3], t2[3], t3[3];
    quatToMatrix(q, m);
    multVecMatrix(t0, c0, m);
    multVecMatrix(t1, c1, m);
    multVecMatrix(t2, c2, m);
    multVecMatrix(t3, c3, m);
        
    glPushMatrix();
    glRotatef(90, 0, 1, 0);
    glLineWidth(2);
    glBegin(GL_LINES);
    glVertex3fv(t0);
    glVertex3fv(t1);
    glVertex3fv(t2);
    glVertex3fv(t3);
    glEnd();
    glPopMatrix();
}

void drawQuaternionControlPoints()
{
    glColor4f(0.0,0.0,1.0,0.8);
    for (unsigned int i=0; i<nPoints; i++) {
        drawQuat(quat[i]);
    }
    glColor4f(1.0,0.0,1.0,0.4);
    for (unsigned int i=0; i<nPoints-1; i++) {
        drawQuat(spline[i].q0);
        drawQuat(spline[i].q1);
        drawQuat(spline[i].c0);
        drawQuat(spline[i].c1);
    }
}

void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* 3D stuff */
    glViewport(0, 0, 640, 640);
    glColor3f(1.0, 1.0, 1.0);

    glPushMatrix();
    glLoadIdentity();
    glRotatef(glat, 1.0, 0.0, 0.0);
    glRotatef(glon, 0.0, 1.0, 0.0);

    drawTeapot();
    drawGlobe();
    drawSpline();
    drawQuaternionControlPoints();
    glPopMatrix();

        
    /* 2D stuff */
    glViewport(640, 0, 260, 640);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glColor4f(1,1,1,1);
    float ypos = 0.9;
    outputString(-1.0,-0.9, "Add      Remove");
    outputString(-1.0,-0.8, "T:");
    outputFloat(0,0, ptens);
    outputString(0,0, " C:");
    outputFloat(0,0, pcont);
    outputString(0,0, " B:");
    outputFloat(0,0, pbias);
    for (unsigned int i=0; i<nPoints; i++) {
        outputInteger(-1.0, ypos, i);
        outputString(0,0, " ");
        outputAngle(0,0,lat[i]);
        outputString(0,0, " ");
        outputAngle(0,0,lon[i]);
        outputString(0,0, " ");
        outputAngle(0,0,hea[i]);
        ypos -= 0.08;
    }

    glDisable(GL_TEXTURE_2D);

    glutSwapBuffers();
}

void reshape(int width, int height)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void animate(int p)
{
    t += 0.02;
    if (t > 1.0) t = 0.0;
    display();
    glutTimerFunc(timeDelay,animate,0);
}

/* Stuff to do with processing the mouse movement */

void globemove(int x, int y)
{
    glon += (mousedown_x - x);
    glon = fmodf(glon, 360);
    glon = (lon < 0) ? glon+360 : glon;
    
    glat += (mousedown_y - y);
    glat= (glat> 90) ? 90 : glat;
    glat = (glat < -90) ? -90 : glat;
    display();
    
    mousedown_x = x;
    mousedown_y = y;
}

void menumove(int x, int y)
{
    if (mousedown_line < nPoints && mousedown_line>=0) {
        switch (mousedown_col) {
            case 0:
                lat[mousedown_line] += (y - mousedown_y) / 100.0;
                break;
            case 1:
                lon[mousedown_line] += (y - mousedown_y) / 100.0;
                break;
            case 2:
                hea[mousedown_line] += (y - mousedown_y) / 100.0;
                break;
        }
        eulerToQuat(lon[mousedown_line], lat[mousedown_line], hea[mousedown_line], quat[mousedown_line]);
    }
    else if (mousedown_line == 22) {
        switch (mousedown_col) {
        case 0:
            ptens += (y - mousedown_y) / 100.0;
            ptens = (ptens > +1.0) ? +1.0 : ptens;
            ptens = (ptens < -1.0) ? -1.0 : ptens;
            break;
        case 1:
            pcont += (y - mousedown_y) / 100.0;
            pcont = (pcont > +1.0) ? +1.0 : pcont;
            pcont = (pcont < -1.0) ? -1.0 : pcont;
            break;
        case 2:
            pbias += (y - mousedown_y) / 100.0;
            pbias = (pbias > +1.0) ? +1.0 : pbias;
            pbias = (pbias < -1.0) ? -1.0 : pbias;
            break;
        }
    } else {
        return;
    }
    createSplines();
    display();
    mousedown_x = x;
    mousedown_y = y;
}

void mousebutton(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON && state==GLUT_DOWN)
    {
        if (x < 640) /* globe */
        {
            mousedown_x = x;
            mousedown_y = y;
            glutMotionFunc(globemove);
        }
        else /* controls */
        {
            mousedown_x = x;
            mousedown_y = y;
            mousedown_line = ((y-10) / (640/25));
            mousedown_col = (x-680) / (40);

            if (mousedown_line == 23)
            {
                if (mousedown_col == 0)
                {
                    if (nPoints < 16)
                    {
                        nPoints += 1;
                        lat[nPoints-1] = 0.0;
                        lon[nPoints-1] = 0.0;
                        hea[nPoints-1] = 0.0;
                        eulerToQuat(lon[nPoints-1], lat[nPoints-1], hea[nPoints-1], quat[nPoints-1]);
                    }
                }
                else
                {
                    if (nPoints > 1)
                    {
                        nPoints -= 1;
                    }
                }
                display();
            }
            else
            {
                glutMotionFunc(menumove);
            }
        }
    }
    else
    {
        glutMotionFunc(NULL);
        createSplines();
    }
}

int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowPosition(200, 180);
    glutInitWindowSize(900, 640);
    
    glutCreateWindow("Quaternion Teapot");
    init(); 
    glutReshapeFunc(reshape);
    glutDisplayFunc(display);
    glutMouseFunc(mousebutton);
    glutTimerFunc(timeDelay,animate,0);

    glutMainLoop();
        
    return 0;
}


