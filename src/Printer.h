#ifndef PRINTER_H
#define PRINTER_H
#include <string>
using namespace std;

class Printer {
    public:
        Printer() { _message = "I am a Printer"; }
        ~Printer() {}
        void print();
    private:
        string _message;
};

#endif