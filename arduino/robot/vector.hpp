#ifndef VECTOR_H
#define VECTOR_H

#include <Arduino.h>

template <class _T>
class vector{
	public:
		vector();
		vector(_T *dataBegin, _T *dataEnd);
		vector(const vector& other); // copy constructor
		vector(vector&& other); // move constructor
		~vector();

		void push_back(_T inputData);
		void clear();
		_T &at(size_t i);
		_T *begin();
		_T *end();
		size_t size();

		_T &operator[](size_t i);
		vector& operator=(const vector& other); // copy assignment
		vector& operator=(vector&& other); // move assignment
	
	private:
		size_t DEFAULT_BUFFER_SIZE = 50;
		size_t _buffer_size = DEFAULT_BUFFER_SIZE;
		size_t _size = 0;

		_T *_data;

};

#endif