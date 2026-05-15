#include "SquaresSequence.h"

SquaresSequence::SquaresSequence(float size, int count) : _size(size), _count(count) {}

void SquaresSequence::update(MotionController& mc, Navigation& nav) {
    switch (_subStep) {
        case 0: mc.moveTo(-_size/2, -_size/2); _subStep++; break;
        case 1: mc.moveTo(_size/2, -_size/2); _subStep++; break;
        case 2: mc.moveTo(_size/2, _size/2); _subStep++; break;
        case 3: mc.moveTo(-_size/2, _size/2); _subStep++; break;
        case 4: 
            mc.moveTo(-_size/2, -_size/2); 
            _size += Config::SQUARE_GROWTH;
            _count--;
            _subStep = (_count > 0) ? 0 : 5; 
            break;
    }
}

bool SquaresSequence::isFinished() const {
    return _count <= 0 && _subStep >= 5;
}