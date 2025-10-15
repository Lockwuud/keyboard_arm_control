#ifndef KEYBOARD_READER_H
#define KEYBOARD_READER_H

#include <termios.h>
#include <unistd.h>
#include <functional>
#include <map>
#include <thread>
#include <atomic>

class KeyboardReader {
public:
    KeyboardReader();
    ~KeyboardReader();
    
    void startListening();
    void stopListening();
    void registerKeyCallback(char key, std::function<void()> callback);
    
private:
    void keyboardLoop();
    
    struct termios original_termios_;
    std::map<char, std::function<void()>> key_callbacks_;
    std::thread keyboard_thread_;
    std::atomic<bool> running_{false};
};

#endif