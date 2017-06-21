#include "HardwareServices/CAN_Services/N2kMsg.h"
#include "HardwareServices/CAN_Services/CANService.h"

#include "Nodes/CANFeedbackReceiver.h"
#include "Tests/unit-tests/TestMocks/MessageVerifier.h"
#include "MessageBus/MessageBus.h"


#include <thread>
#include <chrono>
    
MessageBus msgBus;

void runMessageLoop() {
    msgBus.run();
}

int main() {

    CANService canService;

    CANFeedbackReceiver receiver(msgBus, canService);
    MessageVerifier verifier(msgBus);
    std::thread t1(runMessageLoop);
    t1.detach();
    auto f = canService.start();
    canService.SetLoopBackMode();

    float ratio = 65535 / 60;
    uint16_t rudderFeedback = 21 * ratio;
    uint16_t wingsailFeedback = 13 * ratio;
    uint16_t windvaneSteerAngle = 2 * ratio;
    CanMsg Cmsg;
    Cmsg.id = 701;
    Cmsg.header.ide = 0;
    Cmsg.header.length = 7;
    
    (Cmsg.data[0] = rudderFeedback & 0xff);
    (Cmsg.data[1] = rudderFeedback >> 8);
    (Cmsg.data[2] = wingsailFeedback & 0xff);
    (Cmsg.data[3] = wingsailFeedback >> 8);
    (Cmsg.data[4] = windvaneSteerAngle & 0xff);
    (Cmsg.data[5] = windvaneSteerAngle >> 8);
    (Cmsg.data[6] = 0);

    canService.sendCANMessage(Cmsg);

    const ASPireActuatorFeedbackMsg otherMsg(rudderFeedback, wingsailFeedback, windvaneSteerAngle, 0);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(750));
    std::cout << verifier.verifyActuatorFeedbackMsg(&otherMsg) << std::endl;

    canService.stop();
    f.get();

}