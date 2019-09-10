#include "FileManipulator.h"


FileManipulator::FileManipulator(const char* _fileName){
    importData(_fileName);
}

FileManipulator::FileManipulator(){
}

int FileManipulator::importData(const char* _fileName){
    accessible = SD.begin();

    fileData = "";

    dataNum = 0;

    if(accessible){
        File file = SD.open(_fileName, FILE_READ);

        if(file){
            while(file.available()){
                char c = file.read();
                static char preC = c;

                switch(c){
                    case '/':   //コメントアウト
                        while((char)file.read()!='\n');
                        break;
                    case '\n':  //コンマに置き換える すべてのデータはコンマ区切り
                    case ' ':
                    case ':':
                    case ',':
                        c = ',';
                        if(preC == ',') break;  //コンマが連続で続かない
                    default:
                        fileData += c;
                        dataNum++;
                }

                preC = c;
            }
        }
        else return -2;
        
        cursor = 0;
        cursor2 = -1;

        file.close();
    }
    else {
        return -1;
    }
    return 0;
}


String FileManipulator::readData(){
    cursor = cursor2;
    cursor2 = (int)fileData.indexOf(',', cursor + 1);
    if(cursor2 == -1)return "\0";

    return fileData.substring(cursor + 1, cursor2 - 1);
}

char FileManipulator::readChar(){
    return (readData())[0];
}

int FileManipulator::readInt(){
    return readData().toInt();
}

double FileManipulator::readDouble(){
    return (double)(readData().toFloat());
}
