#include "BaseConverter.h"
#include <cstring>

BaseConverter::BaseConverter(int size) : size(size)
{
  data1 = new unsigned char[size];
  data2 = new unsigned char[size];
  clear();
}

BaseConverter::~BaseConverter()
{
  delete[] data1;
  delete[] data2;
}

void BaseConverter::clear()
{
  memset(data1, 0, sizeof(unsigned char)*size);
}

unsigned char* BaseConverter::getData()
{
  return data1;
}

int BaseConverter::getSize()
{
  return size;
}

unsigned char& BaseConverter::operator[](int index)
{
  return data1[index];
}

void BaseConverter::add(unsigned char* data, int size, int base, 
                        int summand)
{
  int carry = summand;
  for(int i=0; i<size; i++)
  {
    int v = (int)data[i] + carry;
    carry = v / base;
    data[i] = v % base;
  }
}

void BaseConverter::multiply(unsigned char* data, int size, int base, 
                             int factor)
{
  int carry = 0;
  for(int i=0; i<size; i++)
  {
    int v = (int)data[i] * factor + carry;
    carry = v / base;
    data[i] = v % base;
  }  
}

void BaseConverter::convert(int fromBase, int toBase)
{
  //convert
  memset(data2, 0, sizeof(unsigned char)*size);
  for(int i=size-1; i>=0; i--)
  {
    multiply(data2, size, toBase, fromBase);
    add(data2, size, toBase, data1[i]);
  }
  //copy to first buffer
  memcpy(data1, data2, sizeof(unsigned char)*size);
}


