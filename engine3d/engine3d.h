#ifndef LORENZ_ENGINE_3D_H
#define LORENZ_ENGINE_3D_H

// ------------------------------------------------------
// Structs
// ------------------------------------------------------

typedef struct Vector3 {
    double x;
    double y;
    double z;
} Vec3;

typedef struct Vector4 {
    double x;
    double y;
    double z;
    double w;
} Vec4;

typedef struct Matrix4x4 {
    double mat[4][4]; // [row][column]
} Mat4;

typedef struct Plane3 {
    Vec3 position;
    Vec3 normal;
} Plane;

// ------------------------------------------------------
// Functions
// ------------------------------------------------------

// -- Debug --
void showMatrix(const Mat4 matrix);

// -- Matrix Initialization --
Mat4 matrixFromArray(const double arr[4][4]);
Mat4 makeProjectionMatrix(double FOV, double nearPlane, double farPlane, double aspectRatio);
Mat4 makeIdentityMatrix();
Mat4 makePointAtMatrix(const Vec3 position, const Vec3 target, const Vec3 up);
Mat4 makeTranslationMatrix(const Vec3 translation);
Mat4 makeScalingMatrix(const Vec3 scale);
Mat4 makeXRotationMatrix(double theta);
Mat4 makeYRotationMatrix(double theta);
Mat4 makeZRotationMatrix(double theta);

// -- Matrix Math --
// Matrix -> Matrix transformations
void Mat4MultiplyMat4(Mat4* out, const Mat4 a, const Mat4 b);
Mat4 quickMatrixInverse(const Mat4 m);
// Matrix -> Vector transformations
void Mat4MultiplyVec4(Vec4* out, const Mat4 a, const Vec4 b);
void Mat4MultiplyVec3(Vec3* out, const Mat4 a, const Vec3 b);
// Matrix/Vector -> Screen transoformations
void projectVec3ToScreen(Vec3* out, const Mat4 projectionMatrix, const Vec3 point, const int width, const int height);
void projectVec4ToScreen(Vec3* out, const Mat4 projectionMatrix, const Vec4 point, const int width, const int height);

// -- Vector Math --
void Vec3Add(Vec3* out, const Vec3 a, const Vec3 b); // a + b
void Vec3Subtract(Vec3* out, const Vec3 a, const Vec3 b); // a - b
void Vec3Multiply(Vec3* out, const Vec3 a, const Vec3 b); // a * b
void Vec3Multiply(Vec3* out, const Vec3 a, const Vec3 b); // a / b
double Vec3Dot(const Vec3 a, const Vec3 b); // a • b
double Vec3Magnitude(const Vec3 v); // |v|
void Vec3Normalize(Vec3* v); // v / |v|
void Vec3Negative(Vec3* v); // -v
void Vec3Cross(Vec3* out, const Vec3 a, const Vec3 b); // a × b

// -- Plane/Clipping Math --
int isWithinPlane(const Plane plane, const Vec3 point);
int clipWithinPlane(const Plane plane, Vec3* a, Vec3* b);

#endif