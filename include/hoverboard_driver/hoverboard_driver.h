#include "uart_peripheral.h"

class hoverboard_driver : public uart_peripheral
{
public:
    explicit hoverboard_driver(char* device);
    virtual ~hoverboard_driver();
private:
    void velCallback(const geometry_msgs::Twist &vel);
};