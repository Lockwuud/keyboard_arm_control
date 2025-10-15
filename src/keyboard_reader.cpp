#include "keyboard_reader.h"
#include <iostream>

KeyboardReader::KeyboardReader() {
    // 保存当前终端设置
    tcgetattr(STDIN_FILENO, &original_termios_);
}

KeyboardReader::~KeyboardReader() {
    stopListening();
}

void KeyboardReader::startListening() {
    if (running_) return;
    
    running_ = true;
    keyboard_thread_ = std::thread(&KeyboardReader::keyboardLoop, this);
}

void KeyboardReader::stopListening() {
    running_ = false;
    if (keyboard_thread_.joinable()) {
        keyboard_thread_.join();
    }
    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
}

void KeyboardReader::registerKeyCallback(char key, std::function<void()> callback) {
    key_callbacks_[key] = callback;
}

void KeyboardReader::keyboardLoop() {
    struct termios new_termios = original_termios_;
    new_termios.c_lflag &= ~(ICANON | ECHO);  // 非规范模式，无回显
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    
    std::cout << "键盘控制已启动，按 'h' 查看帮助\n";
    
    while (running_) {
        char c = getchar();
        
        if (key_callbacks_.find(c) != key_callbacks_.end()) {
            key_callbacks_[c]();
        }
        
        // 短暂休眠避免CPU过度占用
        usleep(10000);  // 10ms
    }
}