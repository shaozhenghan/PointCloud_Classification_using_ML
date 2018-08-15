#include <Python.h>
#include "data_aug.h"
#include <iostream>

int main(int argc, char const *argv[])
{
    PyObject *point;
    PyObject *angle;
    PyObject *sigma;
    PyObject *clip;
    PyObject *augmented_point;

    Py_Initialize();
    initdata_aug();
    // 浮点形数据必须写为1.0, 2.0 这样的，否则Py_BuildValue()精度损失导致严重错误
    point = Py_BuildValue("[f,f,f]", 1.0, 2.0, 3.0);
    angle = Py_BuildValue("f", 3.14);
    sigma = Py_BuildValue("f", 0.01);
    clip = Py_BuildValue("f", 0.05);
    augmented_point = augment_data(point, angle, sigma, clip);
    
    float x=0.0, y=0.0, z=0.0;
    PyObject *pValue = PyList_GetItem(augmented_point, 0);
    PyObject *pValue_0 = PyList_GET_ITEM(pValue, 0);
    PyObject *pValue_1 = PyList_GET_ITEM(pValue, 1);
    PyObject *pValue_2 = PyList_GET_ITEM(pValue, 2);

    x = PyFloat_AsDouble(pValue_0);
    y = PyFloat_AsDouble(pValue_1);
    z = PyFloat_AsDouble(pValue_2); 

    std::cout << PyList_Size(pValue) << std::endl;
    std::cout << x << std::endl << y << std::endl << z << std::endl;
    
    Py_Finalize();
    return 0;
}
