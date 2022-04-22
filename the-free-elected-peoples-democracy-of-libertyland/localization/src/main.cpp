#include <Arduino.h>
#include <IrPublisher.h>

ros::NodeHandle nh;
IrPublisher irPublisher;

void setup()
{
    nh.initNode();
    irPublisher.setup(nh);
}

void loop()
{
    irPublisher.loop(nh);

    nh.spinOnce();
}