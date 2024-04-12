/*************************************************
 * Contibutors ::-- Hassan Hosni Abdel_Aleem     *
 *                  Mohamed Khaled Galloul       *
 *                  Ahmed Mohamed Mohamed        *
 *                  Moamen Gamal Ashmawy         *
 *                  Youssef Mahmoud Elhely       *
**************************************************/

#include <GL/glut.h>
#include <math.h>
#include <stdio.h>

GLfloat body = 0, shoulder1 = 0,shoulder10 = 0, shoulder2 = 0, shoulder20 = 0, elbow1 = 0, elbow2 = 0, leg1 = 0,leg1z = 0, knee1 = 0, leg2 = 0,leg2z = 0, knee2 = 0;
GLfloat moving, startx, starty, theta = 0.0;
double eye[] = { 0 , 0, -1 };
double center[] = {0, 0, 4};
double up[] = { 0, 1, 0};
double perpendicularAxis[] = { 0, 0, 0 };
GLfloat Znear = 100.0;
int windowWidth;
int windowHeight;
int fingerBase1[5] = { 0 }, fingerUp1[5] = { 0 };
int fingerBase2[5] = { 0 }, fingerUp2[5] = { 0 };

GLfloat angle = 0.0;   /* in degrees */
GLfloat angle2 = 0.0;   /* in degrees */

void rotatePoint(double a[], double theta, double p[])
{

    double temp[3];
    temp[0] = p[0];
    temp[1] = p[1];
    temp[2] = p[2];

    temp[0] = -a[2] * p[1] + a[1] * p[2];
    temp[1] = a[2] * p[0] - a[0] * p[2];
    temp[2] = -a[1] * p[0] + a[0] * p[1];

    temp[0] *= sin(theta);
    temp[1] *= sin(theta);
    temp[2] *= sin(theta);

    temp[0] += (1 - cos(theta)) * (a[0] * a[0] * p[0] + a[0] * a[1] * p[1] + a[0] * a[2] * p[2]);
    temp[1] += (1 - cos(theta)) * (a[0] * a[1] * p[0] + a[1] * a[1] * p[1] + a[1] * a[2] * p[2]);
    temp[2] += (1 - cos(theta)) * (a[0] * a[2] * p[0] + a[1] * a[2] * p[1] + a[2] * a[2] * p[2]);

    temp[0] += cos(theta) * p[0];
    temp[1] += cos(theta) * p[1];
    temp[2] += cos(theta) * p[2];

    p[0] = temp[0];
    p[1] = temp[1];
    p[2] = temp[2];

}

void crossProduct(double a[], double b[], double c[])
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

void normalize(double a[])
{
    double norm;
    norm = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    norm = sqrt(norm);
    a[0] /= norm;
    a[1] /= norm;
    a[2] /= norm;
}


void init(void)
{
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_FLAT);
}
#define ARM1 1
#define ARM2 0

void displayfingers(int armNumber) {
    float y = -0.3;
    for (int i = 0; i < 5; i++) {
        glPushMatrix();
        glTranslatef(armNumber? 1.0:-1.6 , (i ? 0.3 : -0.2), (i ? (y + i * 0.18) : (y + i * 0.15))); // one step forward and one step in y according to finger number
        glRotatef((GLfloat) fingerBase1[i], 0.0, 0.0, i ? -1.0 : 1.0); // rotate figerbase around -z (variable)
        glTranslatef(0.15, 0.0, 0.0);                   // step 0.15 forward
        glPushMatrix();
        glScalef(0.3, 0.1, 0.1);
        glutWireCube(1);                            // DRAW THE FINGERBASE
        glPopMatrix();

        //Draw finger flang 1 
        glTranslatef(0.15, 0.0, 0.0);                   // step 0.15 forward
        glRotatef((GLfloat) -fingerUp1[i], 0.0, 0.0, i ? -1.0 : -1.0);    // rotate upperfinger around -z (variable)
        glTranslatef(0.15, 0.0, 0.0);                   // step 0.15 forward
        glPushMatrix();
        glScalef(0.3, 0.1, 0.1);
        glutWireCube(1);                            // DRAW THE UPPERFINGER
        glPopMatrix();
        glPopMatrix();
    }
}


void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT);
    glPushMatrix();             /* DRAWING STACK */
    gluLookAt(eye[0], eye[1], eye[2], center[0], center[1], center[2], up[0], up[1], up[2]);
    glScalef(0.7, 0.7, 0.7);
    glRotatef(angle2, 1.0, 0.0, 0.0);
    glRotatef(angle, 0.0, 1.0, 0.0);
    glTranslatef(0.0, 2.0, 1.0);
    glPushMatrix();             /* BODY STACK */
    glScalef(2.0, 3.0, 1.0);    
    glutWireCube(1.0);          /* drawing the actual body center */
    glPopMatrix();              /* BODY STACK */

    /*******************************************************
     *                        HEAD                         *
    ********************************************************/
    glPushMatrix();             /* HEAD STACK */
    glTranslatef(0.0, 2.5, 0.0);
    glutSolidSphere(1.0, 50, 50); /* drawing the actual head */
    glPopMatrix();              /* HEAD STACK */

    /*******************************************************
     *                        SHOULDER 1                   *
    ********************************************************/
    glPushMatrix();             /* ARM 1 STACK */
    glTranslatef(1.5, 1.2, 0.0);
    glRotatef((GLfloat)shoulder1, 0.0, 0.0, 1.0);
    glTranslatef(0.0, 0.0, 0.0);
    glRotatef((GLfloat)shoulder10, 0.0, 1.0, 0.0);
    glTranslatef(1.0, 0.0, 0.0);
    glPushMatrix();             /* SHOULDER 1 STACK */
    glScalef(2.0, 0.6, 1.0);
    glutWireCube(1.0);          /* drawing the actual shoulder */
    glPopMatrix();              /* SHOULDER 1 STACK */

    /*******************************************************
     *                        ELBOW 1                      *
    ********************************************************/
    glPushMatrix();             /* ELBOW 1 STACK */
    glTranslatef(1.0, 0.0, 0.0);
    glRotatef((GLfloat)elbow1, 0.0, 0.0, 1.0);
    glTranslatef(1.0, 0.0, 0.0);
    glPushMatrix();             /* ELBOW DRAW STACK */
    glScalef(2.0, 0.6, 1.0);
    glutWireCube(1.0);          /* drawing the actual elbow */
    glPopMatrix();              /* ELBOW DRAW STACK */

    /********************************************************
     *                        FINGERS 1                     *
     ********************************************************/
    displayfingers(ARM1);
    glPopMatrix();              /* ELBOW 1 STACK */
    glPopMatrix();              /* ARM 1 STACK */

    /*******************************************************
     *                        SHOULDER 2                   *
    ********************************************************/
    glPushMatrix();             /* ARM 2 STACK */
    glTranslatef(-1.5, 1.2, 0.0);
    glRotatef((GLfloat)shoulder2, 0.0, 0.0, 1.0);
    glTranslatef(0.0, 0.0, 0.0);
    glRotatef((GLfloat)shoulder20, 0.0, 1.0, 0.0);
    glTranslatef(-1.0, 0.0, 0.0);
    glPushMatrix();             /* SHOULDER 2 STACK */
    glScalef(2.0, 0.6, 1.0);
    glutWireCube(1.0);          /* drawing the actual shoulder */
    glPopMatrix();              /* SHOULDER 2 STACK */

    /*******************************************************
     *                        ELBOW 2                      *
    ********************************************************/
    glPushMatrix();             /* ELBOW 2 STACK */
    glTranslatef(-1.0, 0.0, 0.0);
    glRotatef((GLfloat)elbow2, 0.0, 0.0, 1.0);
    glTranslatef(-1.0, 0.0, 0.0);
    glPushMatrix();             /* ELBOW DRAW STACK */
    glScalef(2.0, 0.6, 1.0);
    glutWireCube(1.0);          /* drawing the actual elbow */
    glPopMatrix();              /* ELBOW DRAW STACK */

    /********************************************************
     *                        FINGERS 2                     *
     ********************************************************/
    displayfingers(ARM2);
    glPopMatrix();              /* ELBOW 2 STACK */
    glPopMatrix();              /* ARM 2 STACK */

    /*******************************************************
     *                        LEG 1                        *
    ********************************************************/
    glPushMatrix();             /* LEG 1 STACK */
    glTranslatef(1.0, -1.35, 0.0);
    glRotatef((GLfloat)leg1, 1.0, 0.0, 0.0);
    glRotatef((GLfloat)leg1z, 0.0, 0.0, 1.0);
    glTranslatef(-0.35, -1.2, 0.0);
    glPushMatrix();             /* THIGH 1 STACK */
    glScalef(0.7, 2.0, 1.0);
    glutWireCube(1.0);          /* drawing the actual leg */
    glPopMatrix();              /* THIGH 1 STACK */

    /*******************************************************
     *                        KNEE 1                       *
    ********************************************************/
    glPushMatrix();             /* LOWER LEG 1 STACK */
    glTranslatef(1.0, -1.35, 0.0);
    glRotatef((GLfloat)knee1, 1.0, 0.0, 0.0);
    glTranslatef(-1, -0.7, 0.0);
    glPushMatrix();             /* KNEE 1 STACK */
    glScalef(0.7, 2.0, 1.0);
    glutWireCube(1.0);          /* drawing the actual knee */
    glPopMatrix();              /* KNEE 1 STACK */
    glTranslatef(0.0, -1.3, 0.0);
    glScalef(0.8, 0.7, 2.0);
    glutSolidCube(1.0);         /* drawing the actual ankle */
    glPopMatrix();              /* LOWER LEG 1 STACK */
    glPopMatrix();              /* LEG 1 STACK */

    /*******************************************************
     *                        LEG 2                        *
    ********************************************************/
    glPushMatrix();             /* LEG 2 STACK */
    glTranslatef(1.0, -1.35, 0.0);
    glRotatef((GLfloat)leg2, 1.0, 0.0, 0.0);
    glTranslatef(-1.2, -0.0, 0.0);
    glRotatef((GLfloat)leg2z, 0.0, 0.0, 1.0);
    glTranslatef(1.0, 0.0, 0.0);
    glTranslatef(-1.53, -1.2, 0.0);
    glPushMatrix();             /* THIGH 2 STACK */
    glScalef(0.7, 2.0, 1.0);
    glutWireCube(1.0);          /* drawing the actual leg */
    glPopMatrix();              /* THIGH 2 STACK */

    /*******************************************************
     *                        KNEE 2                       *
    ********************************************************/
    glPushMatrix();             /* LOWER LEG 2 STACK */
    glTranslatef(1.0, -1.35, 0.0);
    glRotatef((GLfloat)knee2, 1.0, 0.0, 0.0);
    glTranslatef(-1, -0.7, 0.0);
    glPushMatrix();             /* KNEE 2 STACK */
    glScalef(0.7, 2.0, 1.0);
    glutWireCube(1.0);          /* drawing the actual knee */
    glPopMatrix();              /* KNEE 2 STACK */
    glTranslatef(0.0, -1.3, 0.0);
    glScalef(0.8, 0.7, 2.0);
    glutSolidCube(1.0);          /* drawing the actual ankle */
    glPopMatrix();              /* LOWER LEG 2 STACK */
    glPopMatrix();              /* LEG 2 STACK */

    glPopMatrix();              /* DRAWING STACK */
    glutSwapBuffers(); /* Swap buffers after creating the draw */
}

void reshape(int w, int h)
{
    windowWidth = w;
    windowHeight = h;
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    printf("\n%f", Znear);
    gluPerspective(Znear, (GLfloat)w / (GLfloat)h, 0.0, 20.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0, 0.0, -5.0);
}
// control the movement of fingers and shoulder and elbow joints
void keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
    case 'w':
        crossProduct(up, center, perpendicularAxis);
        normalize(perpendicularAxis);
        rotatePoint(perpendicularAxis, 0.05, center);
        rotatePoint(perpendicularAxis, 0.05, up);
        glutPostRedisplay();
        break;
    case 's':
        crossProduct(up, center, perpendicularAxis);
        normalize(perpendicularAxis);
        rotatePoint(perpendicularAxis, -0.05, center);
        rotatePoint(perpendicularAxis, -0.05, up);
        glutPostRedisplay();
        break;
    case 'a':
        rotatePoint(up, -0.05, center);
        glutPostRedisplay();
        break;
    case 'd':
        rotatePoint(up, 0.05, center);
        glutPostRedisplay();
        break;
    case 'q':
        if (Znear > 120) { Znear = 120; }
        else if (Znear <= 60) { Znear = 60; }
        else 
            Znear -= 1.0;

        reshape(windowWidth, windowHeight);
        glutPostRedisplay();
        break;
    case 'Q':
        if (Znear >= 120) { Znear = 120; }
        else if (Znear < 60) { Znear = 60; }
        else
            Znear += 1.0;
        reshape(windowWidth, windowHeight);
        glutPostRedisplay();
        break;
    case '7':
        if (shoulder10 > 150){ shoulder10 = 150; }
        else
        {
            shoulder10 = (shoulder10 + 5);
            glutPostRedisplay();
        }
        break;
    case '8':
        if (shoulder10 <= -90)
        {
            shoulder10 = -90;
        }
        else
        {
            shoulder10 = (shoulder10 - 5);
            glutPostRedisplay();
        }
        break;
        ////////
    case 'p':
        if (shoulder1 > 45){ shoulder1 = 45; }
        else
        {
            shoulder1 = (shoulder1 + 5);
            glutPostRedisplay();
        }
        break;
    case 'P':
        if (shoulder1 <= -90)
        {
            shoulder1 = -90;
        }
        else
        {
            shoulder1 = (shoulder1 - 5);
            glutPostRedisplay();
        }
        break;
        ///////////
    case 'o':
        if (shoulder2 <= -45)
        {
            shoulder2 = -45;
        }
        else
        {
            shoulder2 = (shoulder2 - 5);
            glutPostRedisplay();
        }
        break;
    case 'O':
        if (shoulder2 > 90)
        {
            shoulder2 = 90;
        }
        else
        {
            shoulder2 = (shoulder2 + 5);
            glutPostRedisplay();
        }
        break;
        ////
    case '9':
        if (shoulder20 <= -150)
        {
            shoulder20 = -150;
        }
        else
        {
            shoulder20 = (shoulder20 - 5);
            glutPostRedisplay();
        }
        break;
    case '6':
        if (shoulder20 > 90)
        {
            shoulder20 = 90;
        }
        else
        {
            shoulder20 = (shoulder20 + 5);
            glutPostRedisplay();
        }
        break;
        ////

    case 'i':
        if (elbow1 >= 150)
        {
            elbow1 = 150;
        }
        else
        {
            elbow1 = (elbow1 + 5);
            glutPostRedisplay();
        }
        break;
    case 'I':
        if (elbow1 <= 0)
        {
            elbow1 = 0;
        }
        else
        {
            elbow1 = (elbow1 - 5);
            glutPostRedisplay();
        }
        break;
    case 'u':
        if (elbow2 >= 150)
        {
            elbow2 = 150;
        }
        else
        {
            elbow2 = (elbow2 + 5);
            glutPostRedisplay();
        }
        break;
    case 'U':
        if (elbow2 <= 0)
        {
            elbow2 = 0;
        }
        else
        {
            elbow2 = (elbow2 - 5);
            glutPostRedisplay();
        }
        break;
    case 'y':
        if (leg1 >= 90)
        {
            leg1 = 90;
        }
        else
        {
            leg1 = (leg1 + 5);
            glutPostRedisplay();
        }
        break;
    case 'Y':
        if (leg1 <= -90)
        {
            leg1 = -90;
        }
        else
        {
            leg1 = (leg1 - 5);
            glutPostRedisplay();
        }
        break;
        //
        case 'z':
        if (leg1z >= 90)
        {
            leg1z = 90;
        }
        else
        {
            leg1z = (leg1z + 5);
            glutPostRedisplay();
        }
        break;
    case 'Z':
        if (leg1z <= -90)
        {
            leg1z = -90;
        }
        else
        {
            leg1z = (leg1z - 5);
            glutPostRedisplay();
        }
        break;
        //
    
    case 't':
        if (knee1 <= -90)
        {
            knee1 = -90;
        }
        else
        {
            knee1 = (knee1 - 5);
            glutPostRedisplay();
        }
        break;
    case 'T':
        if (knee1 >= 0)
        {
            knee1 = 0;
        }
        else
        {
            knee1 = (knee1 + 5);
            glutPostRedisplay();
        }
        break;
    case 'r':
        if (leg2 >= 90)
        {
            leg2 = 90;
        }
        else
        {
            leg2 = (leg2 + 5);
            glutPostRedisplay();
        }
        break;
    case 'R':
        if (leg2 <= -90)
        {
            leg2 = -90;
        }
        else
        {
            leg2 = (leg2 - 5);
            glutPostRedisplay();
        }
        break;
    case '2':
        if (leg2z >= 90)
        {
            leg2z = 90;
        }
        else
        {
            leg2z = (leg2z + 5);
            glutPostRedisplay();
        }
        break;
    case '3':
        if (leg2z <= -90)
        {
            leg2z = -90;
        }
        else
        {
            leg2z = (leg2z - 5);
            glutPostRedisplay();
        }
        break;
        //
    case 'e':
        if (knee2 <= -90)
        {
            knee2 = -90;
        }
        else
        {
            knee2 = (knee2 - 5);
            glutPostRedisplay();
        }
        break;
    case 'E':
        if (knee2 >= 0)
        {
            knee2 = 0;
        }
        else
        {
            knee2 = (knee2 + 5);
            glutPostRedisplay();
        }
        break;
    case 'l':
        if (fingerBase1[0] < 90)
            fingerBase1[0] = (fingerBase1[0] + 5) % 360;
        glutPostRedisplay();
        break;
    case 'L':
        if (fingerBase1[0] > 0)
            fingerBase1[0] = (fingerBase1[0] - 5) % 360;
        glutPostRedisplay();
        break;
    case 'k':
        if (fingerUp1[0] < 90)
            fingerUp1[0] = (fingerUp1[0] + 5) % 360;
        glutPostRedisplay();
        break;
    case 'K':
        if (fingerUp1[0] > 0)
            fingerUp1[0] = (fingerUp1[0] - 5) % 360;
        glutPostRedisplay();
        break;
    case 'j':
        if (fingerBase1[1] < 90)
            fingerBase1[1] = (fingerBase1[1] + 5) % 360;
        glutPostRedisplay();
        break;
    case 'J':
        if (fingerBase1[1] > 0)
            fingerBase1[1] = (fingerBase1[1] - 5) % 360;
        glutPostRedisplay();
        break;
    case 'h':
        if (fingerUp1[1] < 90)
            fingerUp1[1] = (fingerUp1[1] + 5) % 360;
        glutPostRedisplay();
        break;
    case 'H':
        if (fingerUp1[1] > 0)
            fingerUp1[1] = (fingerUp1[1] - 5) % 360;
        glutPostRedisplay();
        break;
    case 'g':
        if (fingerBase1[2] < 90)
            fingerBase1[2] = (fingerBase1[2] + 5) % 360;
        glutPostRedisplay();
        break;
    case 'G':
        if (fingerBase1[2] > 0)
            fingerBase1[2] = (fingerBase1[2] - 5) % 360;
        glutPostRedisplay();
        break;
    case 'f':
        if (fingerUp1[2] < 90)
            fingerUp1[2] = (fingerUp1[2] + 5) % 360;
        glutPostRedisplay();
        break;
    case 'F':
        if (fingerUp1[2] > 0)
            fingerUp1[2] = (fingerUp1[2] - 5) % 360;
        glutPostRedisplay();
        break;
    case 'm':
        if (fingerBase1[3] < 90)
            fingerBase1[3] = (fingerBase1[3] + 5) % 360;
        glutPostRedisplay();
        break;
    case 'M':
        if (fingerBase1[3] > 0)
            fingerBase1[3] = (fingerBase1[3] - 5) % 360;
        glutPostRedisplay();
        break;
    case 'n':
        if (fingerUp1[3] < 90)
            fingerUp1[3] = (fingerUp1[3] + 5) % 360;
        glutPostRedisplay();
        break;
    case 'N':
        if (fingerUp1[3] > 0)
            fingerUp1[3] = (fingerUp1[3] - 5) % 360;
        glutPostRedisplay();
        break;
    case 'b':
        if (fingerBase1[4] < 90)
            fingerBase1[4] = (fingerBase1[4] + 5) % 360;
        glutPostRedisplay();
        break;
    case 'B':
        if (fingerBase1[4] > 0)
            fingerBase1[4] = (fingerBase1[4] - 5) % 360;
        glutPostRedisplay();
        break;
    case 'v':
        if (fingerUp1[4] < 90)
            fingerUp1[4] = (fingerUp1[4] + 5) % 360;
        glutPostRedisplay();
        break;
    case 'V':
        if (fingerUp1[4] > 0)
            fingerUp1[4] = (fingerUp1[4] - 5) % 360;
        glutPostRedisplay();
        break;
    case 27:
        exit(0);
        break;
    default:
        break;
    }
}

void mouse(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            moving = 1;
            startx = x;
            starty = y;
        }
        if (state == GLUT_UP) {
            moving = 0;
        }
    }
}


void motion(int x, int y)
{
    if (moving) {
        /*
        angle = angle + (x - startx);
        angle2 = angle2 + (y - starty);*/
        int dx = x - startx;
        int dy = y - starty;
        rotatePoint(up, -(dx % 360) * 0.01 , center);
        crossProduct(up, center, perpendicularAxis);
        normalize(perpendicularAxis);
        rotatePoint(perpendicularAxis, -dy * 0.01, center);
        rotatePoint(perpendicularAxis, -dy * 0.01, up);
        startx = x;
        starty = y;
        glutPostRedisplay();
    }
}



int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(5000, 5000);
    glutInitWindowPosition(10, 10);
    glutCreateWindow(argv[0]);
    init();
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMainLoop();
    return 0;
}
