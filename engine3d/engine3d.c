#include <stdio.h>
#include <math.h>

#include "../constants.h"
#include "engine3d.h"

Mat4 matrixFromArray(const double arr[4][4]) {
    Mat4 matrix;
    int x, y;
    for (y = 0; y < 4; y++) {
        for (x = 0; x < 4; x++) {
            matrix.mat[y][x] = arr[y][x];
        }
    }
    return matrix;
}

Mat4 makeProjectionMatrix(double FOV, double nearPlane, double farPlane, double aspectRatio) {
    double f = 1.0 / (tan(FOV / 2.0));
    double q = farPlane / (farPlane - nearPlane);

    Mat4 projectionMatrix = matrixFromArray(
        (double[4][4]) {
            {aspectRatio * f, 0, 0             , 0},
            {0              , f, 0             , 0},
            {0              , 0, q             , 1},
            {0              , 0, -nearPlane * q, 0}
        }
    );
    
    return projectionMatrix;
}

Mat4 makeIdentityMatrix() {
    Mat4 identityMatrix = matrixFromArray(
        (double[4][4]) {
            {1, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        }
    );
    
    return identityMatrix;
}

Mat4 makePointAtMatrix(const Vec3 position, const Vec3 target, const Vec3 up) {
    Vec3 newForward, n, newUp, newRight;

    // Forward
    Vec3Subtract(&newForward, target, position);
    Vec3Normalize(&newForward);
    // Up
    double upDotFoward = Vec3Dot(up, newForward);
    n.x = newForward.x * upDotFoward;
    n.y = newForward.y * upDotFoward;
    n.z = newForward.z * upDotFoward;
    Vec3Subtract(&newUp, up, n);
    Vec3Normalize(&newUp);
    // Right
    Vec3Cross(&newRight, newUp, newForward);

    // Full
    Mat4 pointAtMatrix = matrixFromArray(
        (double[4][4]) {
            {newRight.x  , newRight.y  , newRight.z  , 0},
            {newUp.x     , newUp.y     , newUp.z     , 0},
            {newForward.x, newForward.y, newForward.z, 0},
            {position.x  , position.y  , position.z  , 1}
        }
    );

    return pointAtMatrix;
}

Mat4 quickMatrixInverse(const Mat4 m) {
    double p1, p2, p3;
    Vec3 a = {m.mat[0][0], m.mat[0][1], m.mat[0][2]};
    Vec3 b = {m.mat[1][0], m.mat[1][1], m.mat[1][2]};
    Vec3 c = {m.mat[2][0], m.mat[2][1], m.mat[2][2]};
    Vec3 t = {m.mat[3][0], m.mat[3][1], m.mat[3][2]};

    p1 = -Vec3Dot(t, a);
    p2 = -Vec3Dot(t, b);
    p3 = -Vec3Dot(t, c);

    Mat4 inverseMatrix = matrixFromArray(
        (double[4][4]) {
            {a.x, b.x, c.x, 0},
            {a.y, b.y, c.y, 0},
            {a.z, b.z, c.z, 0},
            {p1 , p2 , p3 , 1}
        }
    );
    return inverseMatrix;
}

Mat4 makeTranslationMatrix(const Vec3 translation) {
    Mat4 translationMatrix = matrixFromArray(
        (double[4][4]) {
            {1            , 0            , 0            , 0},
            {0            , 1            , 0            , 0},
            {0            , 0            , 1            , 0},
            {translation.x, translation.y, translation.z, 1},
        }
    );
    
    return translationMatrix;
}

Mat4 makeScalingMatrix(const Vec3 scale) {
    Mat4 translationMatrix = matrixFromArray(
        (double[4][4]) {
            {scale.x, 0      , 0      , 0},
            {0      , scale.y, 0      , 0},
            {0      , 0      , scale.z, 0},
            {0      , 0      , 0      , 1},
        }
    );
    
    return translationMatrix;
}

Mat4 makeXRotationMatrix(double theta) {
    Mat4 rotationMatrix = matrixFromArray(
        (double[4][4]) {
            {1, 0         , 0          , 0},
            {0, cos(theta), -sin(theta), 0},
            {0, sin(theta), cos(theta) , 0},
            {0, 0         , 0          , 1},
        }
    );
    
    return rotationMatrix;
}

Mat4 makeYRotationMatrix(double theta) {
    Mat4 rotationMatrix = matrixFromArray(
        (double[4][4]) {
            {cos(theta) , 0, sin(theta), 0},
            {0          , 1, 0         , 0},
            {-sin(theta), 0, cos(theta), 0},
            {0          , 0, 0         , 1},
        }
    );
    
    return rotationMatrix;
}

Mat4 makeZRotationMatrix(double theta) {
    Mat4 rotationMatrix = matrixFromArray(
        (double[4][4]) {
            {cos(theta), -sin(theta), 0, 0},
            {sin(theta), cos(theta) , 0, 0},
            {0         , 0          , 1, 0},
            {0         , 0          , 0, 1},
        }
    );
    
    return rotationMatrix;
}

void showMatrix(const Mat4 matrix) {
    int x, y;
    for (y = 0; y < 4; y++) {
        printf("\n");
        for (x = 0; x < 4; x++) {
            printf("%lf, ", matrix.mat[y][x]);
        }
    }
}

// B multiplied by A, in other words the A transformation is applied to the B transformation
// []A * []B
void Mat4MultiplyMat4(Mat4* out, const Mat4 a, const Mat4 b) {
    int r, c;
    for (r = 0; r < 4; r++) {
        for (c = 0; c < 4; c++) {
            out->mat[r][c] = 
                (b.mat[r][0] * a.mat[0][c]) +
                (b.mat[r][1] * a.mat[1][c]) +
                (b.mat[r][2] * a.mat[2][c]) +
                (b.mat[r][3] * a.mat[3][c]);
        }
    }
}

void Mat4MultiplyVec4(Vec4* out, const Mat4 a, const Vec4 b) {
    out->x = b.x * a.mat[0][0] + b.y * a.mat[1][0] + b.z * a.mat[2][0] + b.w * a.mat[3][0];
    out->y = b.x * a.mat[0][1] + b.y * a.mat[1][1] + b.z * a.mat[2][1] + b.w * a.mat[3][1];
    out->z = b.x * a.mat[0][2] + b.y * a.mat[1][2] + b.z * a.mat[2][2] + b.w * a.mat[3][2];
    out->w = b.x * a.mat[0][3] + b.y * a.mat[1][3] + b.z * a.mat[2][3] + b.w * a.mat[3][3];
}
// Assumes w to be 1
void Mat4MultiplyVec3(Vec3* out, const Mat4 a, const Vec3 b) {
    out->x = b.x * a.mat[0][0] + b.y * a.mat[1][0] + b.z * a.mat[2][0] + a.mat[3][0];
    out->y = b.x * a.mat[0][1] + b.y * a.mat[1][1] + b.z * a.mat[2][1] + a.mat[3][1];
    out->z = b.x * a.mat[0][2] + b.y * a.mat[1][2] + b.z * a.mat[2][2] + a.mat[3][2];
    // out->w = b.x * a.mat[0][3] + b.y * a.mat[1][3] + b.z * a.mat[2][3] + a.mat[3][3];
}

void projectVec3ToScreen(Vec3* out, const Mat4 projectionMatrix, const Vec3 point, const int width, const int height) {
    // Project
    Vec4 newPoint = {point.x, point.y, point.z, 1};
    Vec4 projected;
    Mat4MultiplyVec4(&projected, projectionMatrix, newPoint);
    
    // Divide by z component
    out->x = (projected.x / projected.w);
    out->y = (projected.y / projected.w);
    out->z = (projected.z / projected.w);

    // Normalize to screen coordinates
    out->x = ((out->x + 1.0) / 2) * width;
    out->y = ((out->y + 1.0) / 2) * height;
    out->z = ((out->z + 1.0) / 2);

    // Invert X and Y (makes more sense mathematically)
    // out->x = width - out->x;
    out->y = height - out->y;
}

void projectVec4ToScreen(Vec3* out, const Mat4 projectionMatrix, const Vec4 point, const int width, const int height) {
    // Project
    Vec4 projected;
    Mat4MultiplyVec4(&projected, projectionMatrix, point);
    
    // Divide by z component
    out->x = (projected.x / projected.w);
    out->y = (projected.y / projected.w);
    out->z = (projected.z / projected.w);

    // Normalize to screen coordinates
    out->x = ((out->x + 1.0) / 2) * width;
    out->y = ((out->y + 1.0) / 2) * height;
    out->z = ((out->z + 1.0) / 2);

    // Invert X and Y
    out->x = width - out->x;
    out->y = height - out->y;
}

void Vec3Add(Vec3* out, const Vec3 a, const Vec3 b) {
    out->x = a.x + b.x;
    out->y = a.y + b.y;
    out->z = a.z + b.z;
}
void Vec3Subtract(Vec3* out, const Vec3 a, const Vec3 b) {
    out->x = a.x - b.x;
    out->y = a.y - b.y;
    out->z = a.z - b.z;
}
void Vec3Multiply(Vec3* out, const Vec3 a, const Vec3 b) {
    out->x = a.x * b.x;
    out->y = a.y * b.y;
    out->z = a.z * b.z;
}
void Vec3Divide(Vec3* out, const Vec3 a, const Vec3 b) {
    out->x = a.x / b.x;
    out->y = a.y / b.y;
    out->z = a.z / b.z;
}
double Vec3Dot(const Vec3 a, const Vec3 b) {
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}
double Vec3Magnitude(const Vec3 v) {
    return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}
void Vec3Normalize(Vec3* v) {
    double magnitude = Vec3Magnitude(*v);
    v->x /= magnitude;
    v->y /= magnitude;
    v->z /= magnitude;
}
void Vec3Negative(Vec3* v) {
    v->x = -v->x;
    v->y = -v->y;
    v->z = -v->z;
}
void Vec3Cross(Vec3* out, const Vec3 a, const Vec3 b) {
    out->x = (a.y * b.z) - (a.z * b.y);
    out->y = (a.z * b.x) - (a.x * b.z);
    out->z = (a.x * b.y) - (a.y * b.x);
}

int isWithinPlane(const Plane plane, const Vec3 point) {
    Vec3 dir;
    Vec3Subtract(&dir, point, plane.position);
    int directionDotNormal = Vec3Dot(plane.normal, dir);
    
    return (directionDotNormal >= 0);
}

// Might be good to have this work on separate out vectors
int clipWithinPlane(const Plane plane, Vec3* a, Vec3* b) {
    int aWithin = isWithinPlane(plane, *a);
    int bWithin = isWithinPlane(plane, *b);

    Vec3 lineDirection;
    Vec3Subtract(&lineDirection, *b, *a);

    // If both are inside the clipping plane, nothing needs to be done
    if (aWithin && bWithin) {
        return 1;
    // If both sides are outside the clipping plane, cull the line
    } else if (!(aWithin || bWithin)) {
        return 0;
    }

    double planeDot = -Vec3Dot(plane.normal, plane.position);
    double aDotNormal = Vec3Dot(plane.normal, *a);
    double bDotNormal = Vec3Dot(plane.normal, *b);
    double scalarLength = (-planeDot - aDotNormal) / (bDotNormal - aDotNormal);
    Vec3 vectorLength;
    vectorLength.x = lineDirection.x * scalarLength;
    vectorLength.y = lineDirection.y * scalarLength;
    vectorLength.z = lineDirection.z * scalarLength;
    Vec3 intersection;
    Vec3Add(&intersection, *a, vectorLength);

    // If a is inside and b is not
    if (aWithin) {
        b->x = intersection.x;
        b->y = intersection.y;
        b->z = intersection.z;
    // If b is inside and a is not
    } else { 
        a->x = intersection.x;
        a->y = intersection.y;
        a->z = intersection.z;
    }

    return 1;
}