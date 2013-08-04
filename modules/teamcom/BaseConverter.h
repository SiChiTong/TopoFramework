#ifndef BASECONVERTER_H
#define BASECONVERTER_H


class BaseConverter
{
  private:
    int size;
    unsigned char *data1;
    unsigned char *data2; 

    void add(unsigned char* data, int size, int base, int summand);
    void multiply(unsigned char* data, int size, int base, int factor);
                        
  public:
      
    BaseConverter(int size);
    ~BaseConverter();

    unsigned char* getData();

    void clear();
    int getSize();
    unsigned char& operator[](int index);

    void convert(int fromBase, int toBase);

};


#endif

