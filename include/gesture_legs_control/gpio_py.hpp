#include <Python.h>
#include <iostream>
#include <string>

class GPIOController {
public:
    GPIOController() {
        Py_Initialize();  // 初始化 Python 解释器
    }

    ~GPIOController() {
        Py_Finalize();    // 结束 Python 解释器
    }

    void cleanup() {
        callPythonFunc("Hobot.GPIO", "cleanup");
    }

    void setMode(const std::string& mode) {
        // mode 传 "BOARD" 或 "BCM"
        std::string code = "import Hobot.GPIO as GPIO\n"
                           "GPIO.setmode(GPIO." + mode + ")";
        PyRun_SimpleString(code.c_str());
    }

    void startPWM(int pin, int freq, int duty) {
        std::string code = 
            "import Hobot.GPIO as GPIO\n"
            "pwm = GPIO.PWM(" + std::to_string(pin) + ", " + std::to_string(freq) + ")\n"
            "pwm.start(" + std::to_string(duty) + ")";
        PyRun_SimpleString(code.c_str());
    }

private:
    void callPythonFunc(const std::string& module, const std::string& func) {
        PyObject* pName = PyUnicode_DecodeFSDefault(module.c_str());
        PyObject* pModule = PyImport_Import(pName);
        Py_DECREF(pName);

        if (pModule != nullptr) {
            PyObject* pFunc = PyObject_GetAttrString(pModule, func.c_str());
            if (pFunc && PyCallable_Check(pFunc)) {
                PyObject* pValue = PyObject_CallObject(pFunc, nullptr);
                Py_XDECREF(pValue);
            } else {
                std::cerr << "Function " << func << " not found." << std::endl;
            }
            Py_XDECREF(pFunc);
            Py_DECREF(pModule);
        } else {
            PyErr_Print();
            std::cerr << "Failed to import " << module << std::endl;
        }
    }
};