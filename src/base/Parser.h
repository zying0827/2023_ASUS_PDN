#ifndef PARSER_H
#define PARSER_H

#include "Include.h"
#include "DB.h"

class Parser {
    public:
        Parser(DB& db) : _db(db) {}
        ~Parser() {}

        void testInitialize(double boardWidth, double boardHeight, double gridWidth);
    private:
        DB& _db;

};

#endif