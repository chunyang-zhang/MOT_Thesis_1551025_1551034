#pragma once
#include<iostream>
#include<fstream>
#include"Types.h"
using namespace std;
class OutputFile {
public:
	virtual void output(ofstream& fout) = 0;

};