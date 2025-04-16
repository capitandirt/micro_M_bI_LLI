#ifndef _MY_QUEUE_H_
#define _MY_QUEUE_H_

#include "Arduino.h"
#include "Config.h"

#define MAX_SIZE (MAZE_SIDE_LENGTH * 4)

template<typename T>
class Queue {
public:
    Queue() {
        _begin = 0;
        _end = 0;

        _is_empty = 1;
        _is_full = 0;
    }

    bool isEmpty()
    {
        return _is_empty;
    }

    bool isFull()
    {
        return _is_full;
    }

    bool pushBack(T x)
    {
        if(!_is_full){
            _storage[_end] = x;
            _end = (_end + 1) % MAX_SIZE;
            
            _is_empty = 0;
        }
        _is_full = _begin == _end;

        return isFull();
    }

    T popFront()
    {
        if(!_is_empty){
            T x = _storage[_begin];
            _begin = (_begin + 1) % MAX_SIZE;
            
            _is_full = 0;
            
            _is_empty = _begin == _end;
            return x;
        }
    
        return -1;
    }

    void clear()
    {
        _begin = 0;
        _end = 0;
    }

private:
    T _storage[MAX_SIZE];
    int8_t _begin;
    int8_t _end;

    bool _is_empty;
    bool _is_full;
};

#endif // !_MY_QUEUE_H_