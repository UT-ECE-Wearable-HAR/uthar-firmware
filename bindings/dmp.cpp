// I2Cdev library collection - MPU6050 I2C device class, 6-axis MotionApps 2.0 implementation
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 5/20/2013 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <string.h>
#include <pybind11/pybind11.h>
#include "helper_3dmath.h"

uint8_t dmpGetAccel(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    data[0] = (((uint32_t)packet[28] << 24) | ((uint32_t)packet[29] << 16) | ((uint32_t)packet[30] << 8) | packet[31]);
    data[1] = (((uint32_t)packet[32] << 24) | ((uint32_t)packet[33] << 16) | ((uint32_t)packet[34] << 8) | packet[35]);
    data[2] = (((uint32_t)packet[36] << 24) | ((uint32_t)packet[37] << 16) | ((uint32_t)packet[38] << 8) | packet[39]);
    return 0;
}
uint8_t dmpGetAccel(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    data[0] = (packet[28] << 8) | packet[29];
    data[1] = (packet[32] << 8) | packet[33];
    data[2] = (packet[36] << 8) | packet[37];
    return 0;
}
uint8_t dmpGetAccel(VectorInt16 *v, const char* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    v -> x = (packet[28] << 8) | packet[29];
    v -> y = (packet[32] << 8) | packet[33];
    v -> z = (packet[36] << 8) | packet[37];
    return 0;
}
uint8_t dmpGetQuaternion(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    data[0] = (((uint32_t)packet[0] << 24) | ((uint32_t)packet[1] << 16) | ((uint32_t)packet[2] << 8) | packet[3]);
    data[1] = (((uint32_t)packet[4] << 24) | ((uint32_t)packet[5] << 16) | ((uint32_t)packet[6] << 8) | packet[7]);
    data[2] = (((uint32_t)packet[8] << 24) | ((uint32_t)packet[9] << 16) | ((uint32_t)packet[10] << 8) | packet[11]);
    data[3] = (((uint32_t)packet[12] << 24) | ((uint32_t)packet[13] << 16) | ((uint32_t)packet[14] << 8) | packet[15]);
    return 0;
}
uint8_t dmpGetQuaternion(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    data[0] = ((packet[0] << 8) | packet[1]);
    data[1] = ((packet[4] << 8) | packet[5]);
    data[2] = ((packet[8] << 8) | packet[9]);
    data[3] = ((packet[12] << 8) | packet[13]);
    return 0;
}
uint8_t dmpGetQuaternion(Quaternion *q, const char* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    int16_t qI[4];
    uint8_t status = dmpGetQuaternion(qI, (uint8_t*) packet);
    if (status == 0) {
        q -> w = (float)qI[0] / 16384.0f;
        q -> x = (float)qI[1] / 16384.0f;
        q -> y = (float)qI[2] / 16384.0f;
        q -> z = (float)qI[3] / 16384.0f;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}
// uint8_t dmpGet6AxisQuaternion(long *data, const uint8_t* packet);
// uint8_t dmpGetRelativeQuaternion(long *data, const uint8_t* packet);
uint8_t dmpGetGyro(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    data[0] = (((uint32_t)packet[16] << 24) | ((uint32_t)packet[17] << 16) | ((uint32_t)packet[18] << 8) | packet[19]);
    data[1] = (((uint32_t)packet[20] << 24) | ((uint32_t)packet[21] << 16) | ((uint32_t)packet[22] << 8) | packet[23]);
    data[2] = (((uint32_t)packet[24] << 24) | ((uint32_t)packet[25] << 16) | ((uint32_t)packet[26] << 8) | packet[27]);
    return 0;
}
uint8_t dmpGetGyro(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    data[0] = (packet[16] << 8) | packet[17];
    data[1] = (packet[20] << 8) | packet[21];
    data[2] = (packet[24] << 8) | packet[25];
    return 0;
}
uint8_t dmpGetGyro(VectorInt16 *v, const char* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    v -> x = (packet[16] << 8) | packet[17];
    v -> y = (packet[20] << 8) | packet[21];
    v -> z = (packet[24] << 8) | packet[25];
    return 0;
}
// uint8_t dmpSetLinearAccelFilterCoefficient(float coef);
// uint8_t dmpGetLinearAccel(long *data, const uint8_t* packet);
uint8_t dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
    // get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
    v -> x = vRaw -> x - gravity -> x*8192;
    v -> y = vRaw -> y - gravity -> y*8192;
    v -> z = vRaw -> z - gravity -> z*8192;
    return 0;
}
// uint8_t dmpGetLinearAccelInWorld(long *data, const uint8_t* packet);
uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
    // rotate measured 3D acceleration vector into original state
    // frame of reference based on orientation quaternion
    memcpy(v, vReal, sizeof(VectorInt16));
    v -> rotate(q);
    return 0;
}
// uint8_t dmpGetGyroAndAccelSensor(long *data, const uint8_t* packet);
// uint8_t dmpGetGyroSensor(long *data, const uint8_t* packet);
// uint8_t dmpGetControlData(long *data, const uint8_t* packet);
// uint8_t dmpGetTemperature(long *data, const uint8_t* packet);
// uint8_t dmpGetGravity(long *data, const uint8_t* packet);
uint8_t dmpGetGravity(int16_t *data, const uint8_t* packet) {
    /* +1g corresponds to +8192, sensitivity is 2g. */
    int16_t qI[4];
    uint8_t status = dmpGetQuaternion(qI, packet);
    data[0] = ((int32_t)qI[1] * qI[3] - (int32_t)qI[0] * qI[2]) / 16384;
    data[1] = ((int32_t)qI[0] * qI[1] + (int32_t)qI[2] * qI[3]) / 16384;
    data[2] = ((int32_t)qI[0] * qI[0] - (int32_t)qI[1] * qI[1]
	       - (int32_t)qI[2] * qI[2] + (int32_t)qI[3] * qI[3]) / (2 * 16384);
    return status;
}

uint8_t dmpGetGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}
// uint8_t dmpGetUnquantizedAccel(long *data, const uint8_t* packet);
// uint8_t dmpGetQuantizedAccel(long *data, const uint8_t* packet);
// uint8_t dmpGetExternalSensorData(long *data, int size, const uint8_t* packet);
// uint8_t dmpGetEIS(long *data, const uint8_t* packet);

uint8_t dmpGetEuler(float *data, Quaternion *q) {
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi
    return 0;
}
uint8_t dmpGetYawPitchRoll(VectorFloat *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data->z = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data->y = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data->x = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    return 0;
}

namespace py = pybind11;

PYBIND11_MODULE(dmp, m){
    py::class_<Quaternion>(m, "Quaternion")
        .def(py::init<>())
        .def(py::init<float, float, float, float>())
        .def("getProduct", &Quaternion::getProduct)
        .def("getConjugate", &Quaternion::getConjugate)
        .def("getMagnitude", &Quaternion::getMagnitude)
        .def("normalize", &Quaternion::normalize)
        .def("getNormalized", &Quaternion::getNormalized)
        .def_readwrite("w", &Quaternion::w)
        .def_readwrite("x", &Quaternion::x)
        .def_readwrite("y", &Quaternion::y)
        .def_readwrite("z", &Quaternion::z);
    py::class_<VectorInt16>(m, "VectorInt16")
        .def(py::init<>())
        .def(py::init<int16_t, int16_t, int16_t>())
        .def("getMagnitude", &VectorInt16::getMagnitude)
        .def("normalize", &VectorInt16::normalize)
        .def("getNormalized", &VectorInt16::getNormalized)
        .def("rotate", &VectorInt16::rotate)
        .def("getRotated", &VectorInt16::getRotated)
        .def_readwrite("x", &VectorInt16::x)
        .def_readwrite("y", &VectorInt16::y)
        .def_readwrite("z", &VectorInt16::z);
    py::class_<VectorFloat>(m, "VectorFloat")
        .def(py::init<>())
        .def(py::init<float, float, float>())
        .def("getMagnitude", &VectorFloat::getMagnitude)
        .def("normalize", &VectorFloat::normalize)
        .def("getNormalized", &VectorFloat::getNormalized)
        .def("rotate", &VectorFloat::rotate)
        .def("getRotated", &VectorFloat::getRotated)
        .def_readwrite("x", &VectorFloat::x)
        .def_readwrite("y", &VectorFloat::y)
        .def_readwrite("z", &VectorFloat::z);

    m.def("dmpGetQuaternion", py::overload_cast<Quaternion*, const char*>(&dmpGetQuaternion), "get quaternion");
    m.def("dmpGetGravity", py::overload_cast<VectorFloat*, Quaternion*>(&dmpGetGravity), "get gravity");
    m.def("dmpGetYawPitchRoll", &dmpGetYawPitchRoll);
    m.def("dmpGetAccel", py::overload_cast<VectorInt16*, const char*>(&dmpGetAccel), "get accel");
    m.def("dmpGetGyro", py::overload_cast<VectorInt16*, const char*>(&dmpGetGyro), "get accel");
    m.def("dmpGetLinearAccel", &dmpGetLinearAccel);
    m.def("dmpGetLinearAccelInWorld", &dmpGetLinearAccelInWorld);
    m.def("dmpGetEuler", &dmpGetEuler);
}
