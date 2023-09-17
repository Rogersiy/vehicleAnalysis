#ifndef __LOOP_QUEUE_H_
#define __LOOP_QUEUE_H_

#include <iostream>
#include <string>
#include <unistd.h>
#include <stdint.h>
#include "global.h"

#include <string.h>

using namespace std;

template <typename T>
class LoopQueue
{
private:
    T *data;      //存储用的数组
    int capacity; //存放个数
    int head;     //指向队首
    int tail;     //指向队尾
public:
    LoopQueue();           //无参构造
    bool Init(int a);
    LoopQueue(int a);      
    ~LoopQueue();          //析构
    bool isEmpty();        //判断空
    int getSize();         //返回个数
    bool inLoopQueue(T a); //入队
    T outLoopQueue();      //出队
};

template <typename T>
LoopQueue<T>::LoopQueue()
{
}

template <typename T>
bool LoopQueue<T>::Init(int a)
{
    head = 0;
    tail = 0;
    capacity = a;
    data = new T[capacity];
    return true;
}

template <typename T>
LoopQueue<T>::LoopQueue(int a)
{
    head = 0;
    tail = 0;
    capacity = a;
    data = new T[capacity];
}

template <typename T>
LoopQueue<T>::~LoopQueue()
{
    delete[] data;
}

template <typename T>
bool LoopQueue<T>::isEmpty()
{
    if (head == tail)
        return true;
    else
        return false;
}

template <typename T>
int LoopQueue<T>::getSize()
{
    return (tail - head + capacity) % capacity;
}

template <typename T>
bool LoopQueue<T>::inLoopQueue(T in)
{
    if ((tail + 1) % capacity == head)
    {
//        cout << "push err, capacity is " << capacity << ", head is " << head << ",tail is " << tail << ",data is " << in << endl;
//        return false;
        delete data[head];
        head = (head + 1) % capacity;
    }
    data[tail] = in;
    // cout << "push, capacity is " << capacity << ", head is " << head << ",tail is " << tail << ",data is " << data[tail] << endl;
    tail = (tail + 1) % capacity;
    return true;
}

template <typename T>
T LoopQueue<T>::outLoopQueue()
{
    if (tail == head)
    {
        // cout << "top error, capacity is " << capacity << ", head is " << head << ",tail is " << tail << ",data is " << data[head] << endl;
        return NULL;
    }
    // cout << "top, capacity is " << capacity << ", head is " << head << ",tail is " << tail << ",data is " << data[head] << endl;

    int head_before = head;
    head = (head + 1) % capacity;

    return data[head_before % capacity];
}

#if 0
int test_nomal()
{
    LoopQueue<string> queue(6);

    string keyword;
    while (cin >> keyword && keyword[0] != 'q')
    {
        queue.inLoopQueue(keyword);
    }

    cout << "队列长度" << queue.getSize() << endl;
    while (true)
    {
        if (!queue.isEmpty())
        {
            cout << "get top data :" << queue.outLoopQueue() << endl;
        }
        sleep(1);
    }
    // getchar();
    return 0;
}

int main(int argc, char *argv[])
{
    test_nomal();
    return 0;
}
#endif
#endif

