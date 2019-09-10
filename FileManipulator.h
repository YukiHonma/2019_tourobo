#ifndef FILEMANIPULATOR_H
#define FILEMANIPULATOR_H

#include "Arduino.h"
#include "SD.h"
#include "String.h"


//SDカードクラスの拡張 ファイルデータ読込用クラス
class FileManipulator{
public:
    FileManipulator(const char* _fileName);
    FileManipulator();

    int importData(const char* _fileName);

    String readData();
    char readChar();
    int readInt();
    double readDouble();

private:
    bool accessible;
    int cursor;
    int cursor2;
    char *fileName;
    int dataNum;

    String fileData;
};


#endif