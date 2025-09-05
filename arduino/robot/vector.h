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
		size_t DEFAULT_BUFFER_SIZE = 10;
		size_t _buffer_size = DEFAULT_BUFFER_SIZE;
		size_t _size = 0;

		_T *_data;

};

template <class _T>
vector<_T>::vector() {
	_data = new _T[_buffer_size];
}

template <class _T>
vector<_T>::vector(_T* dataBegin, _T* dataEnd) {
	size_t n = static_cast<_T*>(dataEnd) - static_cast<_T*>(dataBegin);
	_buffer_size = (n > 0 ? n: DEFAULT_BUFFER_SIZE);
	_data = new _T[_buffer_size];
	_size = n;

	for (size_t i = 0; i < _size; i++) {
		_data[i] = static_cast<_T*>(dataBegin)[i];
	}
}

// copy constructor
template <class _T>
vector<_T>::vector(const vector& other) {
	_buffer_size = other._buffer_size;
	_size = other._size;
	_data = new _T[_buffer_size];
	for (size_t i = 0; i < _size; i++) {
		_data[i] = other._data[i];
	}
}

// move constructor
template <class _T>
vector<_T>::vector(vector&& other){
	_buffer_size = other._buffer_size;
	_size = other._size;
	_data = other._data;

	other._data = nullptr;
	other._size = 0;
	other._buffer_size = DEFAULT_BUFFER_SIZE;
}

template <class _T>
vector<_T>::~vector() {
	delete[] _data;
}

// copy assignment
template <class _T>
vector<_T>& vector<_T>::operator=(const vector& other) {
	if (this != &other) {
		delete[] _data;
		_buffer_size = other._buffer_size;
		_size = other._size;
		_data = new _T[_buffer_size];
		for (size_t i = 0; i < _size; i++) {
			_data[i] = other._data[i];
		}
	}
	return *this;
}

// move assignment
template <class _T>
vector<_T>& vector<_T>::operator=(vector&& other){
	if (this != &other) {
		delete[] _data;
		_buffer_size = other._buffer_size;
		_size = other._size;
		_data = other._data;

		other._data = nullptr;
		other._size = 0;
		other._buffer_size = DEFAULT_BUFFER_SIZE;
	}
	return *this;
}

template <class _T>
void vector<_T>::push_back(_T inputData) {
	if (_size >= _buffer_size) {
		// grow capacity
		_buffer_size *= 2;
		_T *newData = new _T[_buffer_size];

		for (size_t i = 0; i < _size; i++) {
			newData[i] = _data[i];
		}

		delete[] _data;
		_data = newData;
	}

	_data[_size++] = inputData;
}

template <class _T>
void vector<_T>::clear() {
	_size = 0;
}

template <class _T>
_T& vector<_T>::at(size_t i) {
	if (i >= _size) {
		// Arduino has no exceptions → return last element as fallback
		return _data[_size - 1];
	}
	return _data[i];
}

template <class _T>
_T& vector<_T>::operator[](size_t i) {
	return _data[i]; // unchecked
}

template <class _T>
_T* vector<_T>::begin() {
	return _data;
}

template <class _T>
_T* vector<_T>::end() {
	return _data + _size;
}

template <class _T>
size_t vector<_T>::size() {
	return _size;
}

#endif