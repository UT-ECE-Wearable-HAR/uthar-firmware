// I2Cdev library collection - MPU6050 I2C device class, 6-axis MotionApps 2.0
// implementation Based on InvenSense MPU-6050 register map document rev. 2.0,
// 5/19/2011 (RM-MPU-6000A-00) 5/20/2013 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at
// https://github.com/jrowberg/i2cdevlib
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

#define PY_SSIZE_T_CLEAN
#include "helper_3dmath.h"
#include <Python.h>
#include <numpy/arrayobject.h>
#include <string.h>

uint8_t dmpGetAccel(int16_t *data, const uint8_t *packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default
  // supported now)
  data[0] = (packet[28] << 8) | packet[29];
  data[1] = (packet[32] << 8) | packet[33];
  data[2] = (packet[36] << 8) | packet[37];
  return 0;
}
uint8_t dmpGetAccel(VectorInt16 *v, const uint8_t *packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default
  // supported now)
  v->x = (packet[28] << 8) | packet[29];
  v->y = (packet[32] << 8) | packet[33];
  v->z = (packet[36] << 8) | packet[37];
  return 0;
}
uint8_t dmpGetQuaternion(int16_t *data, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default
  // supported now)
  data[0] = ((packet[0] << 8) | packet[1]);
  data[1] = ((packet[4] << 8) | packet[5]);
  data[2] = ((packet[8] << 8) | packet[9]);
  data[3] = ((packet[12] << 8) | packet[13]);
  return 0;
}
uint8_t dmpGetQuaternion(Quaternion *q, uint8_t *packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default
  // supported now)
  int16_t qI[4];
  uint8_t status = dmpGetQuaternion(qI, packet);
  if (status == 0) {
    q->w = (float)qI[0] / 16384.0f;
    q->x = (float)qI[1] / 16384.0f;
    q->y = (float)qI[2] / 16384.0f;
    q->z = (float)qI[3] / 16384.0f;
    return 0;
  }
  return status; // int16 return value, indicates error if this line is reached
}
uint8_t dmpGetGyro(int16_t *data, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default
  // supported now)
  data[0] = (packet[16] << 8) | packet[17];
  data[1] = (packet[20] << 8) | packet[21];
  data[2] = (packet[24] << 8) | packet[25];
  return 0;
}
uint8_t dmpGetGyro(VectorInt16 *v, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default
  // supported now)
  v->x = (packet[16] << 8) | packet[17];
  v->y = (packet[20] << 8) | packet[21];
  v->z = (packet[24] << 8) | packet[25];
  return 0;
}
// uint8_t dmpSetLinearAccelFilterCoefficient(float coef);
// uint8_t dmpGetLinearAccel(long *data, const uint8_t* packet);
uint8_t dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw,
                          VectorFloat *gravity) {
  // get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet,
  // sensitivity is 2g)
  v->x = vRaw->x - gravity->x * 8192;
  v->y = vRaw->y - gravity->y * 8192;
  v->z = vRaw->z - gravity->z * 8192;
  return 0;
}
// uint8_t dmpGetLinearAccelInWorld(long *data, const uint8_t* packet);
uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal,
                                 Quaternion *q) {
  // rotate measured 3D acceleration vector into original state
  // frame of reference based on orientation quaternion
  memcpy(v, vReal, sizeof(VectorInt16));
  v->rotate(q);
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
  uint8_t status = dmpGetQuaternion(qI, (uint8_t*)packet);
  data[0] = ((int32_t)qI[1] * qI[3] - (int32_t)qI[0] * qI[2]) / 16384;
  data[1] = ((int32_t)qI[0] * qI[1] + (int32_t)qI[2] * qI[3]) / 16384;
  data[2] = ((int32_t)qI[0] * qI[0] - (int32_t)qI[1] * qI[1] -
             (int32_t)qI[2] * qI[2] + (int32_t)qI[3] * qI[3]) /
            (2 * 16384);
  return status;
}

uint8_t dmpGetGravity(VectorFloat *v, Quaternion *q) {
  v->x = 2 * (q->x * q->z - q->w * q->y);
  v->y = 2 * (q->w * q->x + q->y * q->z);
  v->z = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
  return 0;
}
// uint8_t dmpGetUnquantizedAccel(long *data, const uint8_t* packet);
// uint8_t dmpGetQuantizedAccel(long *data, const uint8_t* packet);
// uint8_t dmpGetExternalSensorData(long *data, int size, const uint8_t*
// packet); uint8_t dmpGetEIS(long *data, const uint8_t* packet);

uint8_t dmpGetEuler(float *data, Quaternion *q) {
  data[0] = atan2(2 * q->x * q->y - 2 * q->w * q->z,
                  2 * q->w * q->w + 2 * q->x * q->x - 1); // psi
  data[1] = -asin(2 * q->x * q->z + 2 * q->w * q->y);     // theta
  data[2] = atan2(2 * q->y * q->z - 2 * q->w * q->x,
                  2 * q->w * q->w + 2 * q->z * q->z - 1); // phi
  return 0;
}
uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
  // yaw: (about Z axis)
  data[0] = atan2(2 * q->x * q->y - 2 * q->w * q->z,
                  2 * q->w * q->w + 2 * q->x * q->x - 1);
  // pitch: (nose up/down, about Y axis)
  data[1] = atan(gravity->x /
                 sqrt(gravity->y * gravity->y + gravity->z * gravity->z));
  // roll: (tilt left/right, about X axis)
  data[2] = atan(gravity->y /
                 sqrt(gravity->x * gravity->x + gravity->z * gravity->z));
  return 0;
}

static PyObject *accel(PyObject *self, PyObject *args) {
  PyObject *bytes;
  const char *packet;
  if (!PyArg_ParseTuple(args, "S", &bytes))
    return NULL;
  packet = PyBytes_AsString(bytes);
  npy_intp dim = {3};
  PyArrayObject *accel = (PyArrayObject *)PyArray_SimpleNew(1, &dim, NPY_INT16);
  int16_t *dptr = (int16_t *)PyArray_DATA(accel);
  dmpGetAccel(dptr, (uint8_t *)packet);
  return PyArray_Return(accel);
}

static PyObject *quaternion(PyObject *self, PyObject *args) {
  PyObject *bytes;
  const char *packet;
  if (!PyArg_ParseTuple(args, "S", &bytes))
    return NULL;
  packet = PyBytes_AsString(bytes);
  npy_intp dim = {4};
  PyArrayObject *quat =
      (PyArrayObject *)PyArray_SimpleNew(1, &dim, NPY_FLOAT32);
  float *dptr = (float *)PyArray_DATA(quat);
  Quaternion q;
  dmpGetQuaternion(&q, (uint8_t *)packet);
  dptr[0] = q.w;
  dptr[1] = q.x;
  dptr[2] = q.y;
  dptr[3] = q.z;

  return PyArray_Return(quat);
}

static PyObject *gyro(PyObject *self, PyObject *args) {
  PyObject *bytes;
  const char *packet;
  if (!PyArg_ParseTuple(args, "S", &bytes))
    return NULL;
  packet = PyBytes_AsString(bytes);
  npy_intp dim = {3};
  PyArrayObject *gyro = (PyArrayObject *)PyArray_SimpleNew(1, &dim, NPY_INT16);
  int16_t *dptr = (int16_t *)PyArray_DATA(gyro);
  dmpGetGyro(dptr, (uint8_t *)packet);
  return PyArray_Return(gyro);
}

static PyObject *linAccel(PyObject *self, PyObject *args) {
  PyObject *bytes;
  const char *packet;
  if (!PyArg_ParseTuple(args, "S", &bytes))
    return NULL;
  packet = PyBytes_AsString(bytes);
  npy_intp dim = {3};
  PyArrayObject *linAccel =
      (PyArrayObject *)PyArray_SimpleNew(1, &dim, NPY_INT16);
  int16_t *dptr = (int16_t *)PyArray_DATA(linAccel);
  VectorInt16 accel, lin_accel;
  VectorFloat gravity;
  Quaternion q;
  dmpGetQuaternion(&q, (uint8_t *)packet);
  dmpGetGravity(&gravity, &q);
  dmpGetAccel(&accel, (uint8_t *)packet);
  dmpGetLinearAccel(&lin_accel, &accel, &gravity);
  dptr[0] = lin_accel.x;
  dptr[1] = lin_accel.y;
  dptr[2] = lin_accel.z;
  return PyArray_Return(linAccel);
}

static PyObject *linAccelWorld(PyObject *self, PyObject *args) {
  PyObject *bytes;
  const char *packet;
  if (!PyArg_ParseTuple(args, "S", &bytes))
    return NULL;
  packet = PyBytes_AsString(bytes);
  npy_intp dim = {3};
  PyArrayObject *linAccelWorld =
      (PyArrayObject *)PyArray_SimpleNew(1, &dim, NPY_INT16);
  int16_t *dptr = (int16_t *)PyArray_DATA(linAccelWorld);
  VectorInt16 accel, lin_accel, lin_accel_world;
  VectorFloat gravity;
  Quaternion q;
  dmpGetQuaternion(&q, (uint8_t *)packet);
  dmpGetGravity(&gravity, &q);
  dmpGetAccel(&accel, (uint8_t *)packet);
  dmpGetLinearAccel(&lin_accel, &accel, &gravity);
  dmpGetLinearAccelInWorld(&lin_accel_world, &lin_accel, &q);
  dptr[0] = lin_accel_world.x;
  dptr[1] = lin_accel_world.y;
  dptr[2] = lin_accel_world.z;
  return PyArray_Return(linAccelWorld);
}

static PyObject *gravity(PyObject *self, PyObject *args) {
  PyObject *bytes;
  const char *packet;
  if (!PyArg_ParseTuple(args, "S", &bytes))
    return NULL;
  packet = PyBytes_AsString(bytes);
  npy_intp dim = {3};
  PyArrayObject *gravity =
      (PyArrayObject *)PyArray_SimpleNew(1, &dim, NPY_FLOAT32);
  float *dptr = (float *)PyArray_DATA(gravity);
  VectorFloat g;
  Quaternion q;
  dmpGetQuaternion(&q, (uint8_t *)packet);
  dmpGetGravity(&g, &q);
  dptr[0] = g.x;
  dptr[1] = g.y;
  dptr[2] = g.z;
  return PyArray_Return(gravity);
}

static PyObject *euler(PyObject *self, PyObject *args) {
  PyObject *bytes;
  const char *packet;
  if (!PyArg_ParseTuple(args, "S", &bytes))
    return NULL;
  packet = PyBytes_AsString(bytes);
  npy_intp dim = {3};
  PyArrayObject *euler =
      (PyArrayObject *)PyArray_SimpleNew(1, &dim, NPY_FLOAT32);
  float *dptr = (float *)PyArray_DATA(euler);
  Quaternion q;
  dmpGetQuaternion(&q, (uint8_t *)packet);
  dmpGetEuler(dptr, &q);
  return PyArray_Return(euler);
}

static PyObject *yawPitchRoll(PyObject *self, PyObject *args) {
  PyObject *bytes;
  const char* packet;
  if (!PyArg_ParseTuple(args, "S", &bytes))
    return NULL;
  packet = PyBytes_AsString(bytes);
  npy_intp dim = {3};
  PyArrayObject *yawPitchRoll =
      (PyArrayObject *)PyArray_SimpleNew(1, &dim, NPY_FLOAT32);
  float *dptr = (float *)PyArray_DATA(yawPitchRoll);
  VectorFloat g;
  Quaternion q;
  dmpGetQuaternion(&q, (uint8_t *)packet);
  dmpGetGravity(&g, &q);
  dmpGetYawPitchRoll(dptr, &q, &g);
  return PyArray_Return(yawPitchRoll);
}

static PyMethodDef methods[] = {
    {"accel", accel, METH_VARARGS, "acceleration: i16[3]"},
    {"quaternion", quaternion, METH_VARARGS, "quaternion: f32[4]"},
    {"gyro", gyro, METH_VARARGS, "gyro: i16[3]"},
    {"linAccel", linAccel, METH_VARARGS, "linAccel: i16[3]"},
    {"linAccelWorld", linAccelWorld, METH_VARARGS, "linAccelWorld: i16[3]"},
    {"gravity", gravity, METH_VARARGS, "gravity: f32[3]"},
    {"euler", euler, METH_VARARGS, "euler: f32[3]"},
    {"yawPitchRoll", yawPitchRoll, METH_VARARGS, "yawPitchRoll: f32[3]"},
    {NULL, NULL, 0, NULL} /* Sentinel */
};

static struct PyModuleDef module = {
    PyModuleDef_HEAD_INIT, "dmp", "module to extract mpu data from dmp packet",
    -1, methods};

PyMODINIT_FUNC PyInit_dmp(void) {
  import_array();
  return PyModule_Create(&module);
}
