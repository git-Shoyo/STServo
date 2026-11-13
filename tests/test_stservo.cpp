#include "STServo/stservo.hpp"
#include <cstdint>
#include <cstdio>
#include <deque>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#define CHECK(expr) do { if (!(expr)) throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) + " " #expr); } while (0)
using Bytes = std::vector<std::uint8_t>;
using Result = STServo::Result;
static Bytes status(std::uint8_t id, std::uint8_t error = 0, Bytes data = {}) {
    Bytes out = {255,255,id,static_cast<std::uint8_t>(data.size()+2),error};
    out.insert(out.end(),data.begin(),data.end());
    unsigned sum=0;
    for (std::size_t i=2;i<out.size();++i) sum+=out[i];
    out.push_back(static_cast<std::uint8_t>(255-(sum&255)));
    return out;
}
static Bytes join(Bytes a, const Bytes& b) { a.insert(a.end(),b.begin(),b.end()); return a; }
struct Fake final : stservo::Transport {
    std::uint32_t now=0, written_at=0, delay=0, gap=0, write_time=0;
    bool write_ok=true, clear_ok=true, read_error=false, echo=false;
    std::deque<Bytes> scripts;
    Bytes pending;
    std::size_t position=0, clears=0;
    std::vector<Bytes> writes;
    bool write(const std::uint8_t* data,std::size_t size,std::uint32_t budget) override {
        CHECK(budget>0);
        writes.push_back(Bytes(data,data+size));
        now+=write_time;
        written_at=now;
        position=0;
        pending=echo ? writes.back() : Bytes{};
        if (!scripts.empty()) { pending=join(pending,scripts.front()); scripts.pop_front(); }
        return write_ok;
    }
    int readByte() override {
        if(read_error) return -2;
        if(position==pending.size() || static_cast<std::uint32_t>(now-written_at)<delay+gap*position) return -1;
        return pending[position++];
    }
    bool discardInput() override { ++clears; pending.clear(); position=0; return clear_ok; }
    std::uint32_t nowMs() const override { return now; }
    void idle() override { ++now; }
};
static void multi_id_results() {
    Fake f; f.scripts={status(1,1),status(2)}; STServo bus(f);
    const auto first=bus.ping(1), second=bus.ping(2);
    CHECK(first.ok() && first.status_received && first.id==1 && first.servo_error==1);
    CHECK(second.ok() && second.id==2 && second.servo_error==0);
    CHECK(first.servo_error==1);
    CHECK(f.writes[0]==Bytes({255,255,1,2,1,251}));
    CHECK(f.writes[1]==Bytes({255,255,2,2,1,250}));
}
static void other_id_noise_and_fragments() {
    Fake f; f.gap=1;
    f.scripts={join(Bytes{0,123,255},join(status(2),status(1)))};
    STServo bus(f,100); const auto r=bus.ping(1);
    CHECK(r.ok() && r.id==1 && f.now>0);
}
static void checksum_and_length_recovery() {
    Fake f; auto bad=status(1); bad.back()^=1;
    f.scripts={join(bad,status(1))}; STServo bus(f);
    CHECK(bus.ping(1).ok());
    f.scripts={bad}; CHECK(bus.ping(1).result==Result::CHECKSUM_ERROR);
    f.scripts={join(Bytes{255,255,1,255},status(1))}; CHECK(bus.ping(1).ok());
    f.scripts={status(1,0,{1})}; CHECK(bus.ping(1).result==Result::LENGTH_ERROR);
}
static void timeout_wrap_and_wrong_id() {
    Fake f; f.now=0xfffffffdU; STServo bus(f,5);
    CHECK(bus.ping(1).result==Result::TIMEOUT); CHECK(f.now==2);
    f.scripts={status(2)}; CHECK(bus.ping(1).result==Result::TIMEOUT);
    f.scripts={Bytes{255,255,1,2,0}}; CHECK(bus.ping(1).result==Result::TIMEOUT);
}
static void read_commit_and_servo_error() {
    Fake f; f.scripts={status(2,0,{2})}; STServo bus(f);
    std::uint8_t data=99; auto r=bus.readRegister(2,5,&data,1);
    CHECK(r.ok() && r.status_received && r.parameter_count==1 && data==2);
    CHECK(f.writes.back()==Bytes({255,255,2,4,2,5,1,241}));
    f.scripts={status(2,8)}; data=77; r=bus.readRegister(2,5,&data,1);
    CHECK(r.ok() && r.status_received && r.servo_error==8 && data==77 && r.parameter_count==0);
    f.scripts={status(2,0,{2,3})}; r=bus.readRegister(2,5,&data,1);
    CHECK(r.result==Result::LENGTH_ERROR && data==77 && !r.status_received);
}
static void writes_and_ack_policy() {
    Fake f; STServo bus(f); const std::uint8_t value=1;
    auto r=bus.writeRegister(1,0x21,&value,1);
    CHECK(r.ok() && !r.status_received);
    CHECK(f.writes.back()==Bytes({255,255,1,4,3,0x21,1,0xd5}));
    f.scripts={status(2,4)};
    r=bus.writeRegister(2,0x21,&value,1,STServo::WriteResponse::Required);
    CHECK(r.ok() && r.status_received && r.servo_error==4 && r.id==2);
    r=bus.writeRegister(254,0x21,&value,1); CHECK(r.ok() && !r.status_received);
    CHECK(bus.writeRegister(254,0x21,&value,1,STServo::WriteResponse::Required).result==Result::INVALID_ARGUMENT);
}
static void control_register_helpers() {
    Fake f; STServo bus(f);
    CHECK(bus.setTorqueEnabled(1,true).ok());
    CHECK(f.writes.back()==Bytes({255,255,1,4,3,0x28,1,0xce}));
    f.scripts={status(2)};
    const auto off=bus.setTorqueEnabled(2,false,STServo::WriteResponse::Required);
    CHECK(off.ok() && off.status_received && off.servo_error==0);
    CHECK(f.writes.back()==Bytes({255,255,2,4,3,0x28,0,0xce}));
    f.scripts={status(1,8)};
    const auto mode=bus.setPositionMode(1,STServo::WriteResponse::Required);
    CHECK(mode.ok() && mode.status_received && mode.servo_error==8);
    CHECK(f.writes.back()==Bytes({255,255,1,4,3,0x21,0,0xd6}));
    CHECK(bus.setPositionMode(254).ok());
    CHECK(bus.setTorqueEnabled(254,true,STServo::WriteResponse::Required).result==Result::INVALID_ARGUMENT);
    CHECK(bus.setEepromLocked(1,false).ok()); CHECK(f.writes.back()[5]==0x37 && f.writes.back()[6]==0);
    CHECK(bus.setEepromLocked(1,true).ok()); CHECK(f.writes.back()[6]==1);
}
static void position_targets() {
    Fake f; STServo bus(f);
    auto r=bus.setPosition(1,2048);
    CHECK(r.ok() && !r.status_received);
    CHECK(f.writes.back()==Bytes({255,255,1,5,3,0x2a,0,8,0xc4}));
    CHECK(bus.setPosition(2,0).ok());
    CHECK(f.writes.back()==Bytes({255,255,2,5,3,0x2a,0,0,0xcb}));
    f.scripts={status(2)};
    r=bus.setPosition(2,4095,STServo::WriteResponse::Required);
    CHECK(r.ok() && r.status_received && !r.servo_error && r.id==2);
    CHECK(f.writes.back()==Bytes({255,255,2,5,3,0x2a,255,15,0xbd}));
    f.scripts={status(1,8)};
    r=bus.setPosition(1,2048,STServo::WriteResponse::Required);
    CHECK(r.ok() && r.status_received && r.servo_error==8);
    const auto count=f.writes.size();
    CHECK(bus.setPosition(1,4096).result==Result::INVALID_ARGUMENT);
    CHECK(bus.setPosition(1,65535).result==Result::INVALID_ARGUMENT);
    CHECK(bus.setPosition(255,0).result==Result::INVALID_ARGUMENT);
    CHECK(bus.setPosition(254,0,STServo::WriteResponse::Required).result==Result::INVALID_ARGUMENT);
    CHECK(f.writes.size()==count);
    CHECK(bus.setPosition(254,2048).ok());
    CHECK(f.writes.back()[2]==254);
    f.echo=true; bus.setEchoMode(STServo::EchoMode::LocalEcho);
    CHECK(bus.setPosition(1,1024).ok()); CHECK(f.position==f.pending.size());
}
static void id_change_validation_and_readback() {
    Fake f; STServo bus(f);
    CHECK(bus.setId(1,2).ok());
    CHECK(f.writes.back()==Bytes({255,255,1,4,3,5,2,0xf0}));
    CHECK(f.now==0); // ID-changing WRITE deliberately never waits for an ACK.
    f.scripts={join(status(1),join(status(2),status(2,0,{2})))};
    std::uint8_t actual=99;
    const auto r=bus.readRegister(2,5,&actual,1);
    CHECK(r.ok() && r.status_received && actual==2);
    CHECK(bus.setId(0,253).ok()); CHECK(bus.setId(253,0).ok());
    const auto count=f.writes.size();
    CHECK(bus.setId(254,2).result==Result::INVALID_ARGUMENT);
    CHECK(bus.setId(255,2).result==Result::INVALID_ARGUMENT);
    CHECK(bus.setId(1,254).result==Result::INVALID_ARGUMENT);
    CHECK(bus.setId(1,255).result==Result::INVALID_ARGUMENT);
    CHECK(f.writes.size()==count);
    f.echo=true; bus.setEchoMode(STServo::EchoMode::LocalEcho);
    CHECK(bus.setId(1,2).ok()); CHECK(f.position==f.pending.size());
}
static void sync_velocity_golden_packet() {
    Fake f; STServo bus(f); const std::uint8_t ids[]={1,2}; const std::int32_t values[]={500,-500};
    CHECK(bus.syncSetVelocity(ids,values,2)==Result::OK);
    CHECK(f.writes.back()==Bytes({255,255,254,10,0x83,0x2e,2,1,0xf4,1,2,0xf4,0x81,0xd7}));
    CHECK(f.now==0); // Does not wait for nonexistent broadcast ACKs.
}
static void sync_validation() {
    Fake f; STServo bus(f); std::uint8_t ids[]={1,1}, data[]={1,1};
    CHECK(bus.syncWriteRegister(0x21,1,ids,data,2)==Result::INVALID_ARGUMENT);
    ids[1]=254; CHECK(bus.syncWriteRegister(0x21,1,ids,data,2)==Result::INVALID_ARGUMENT);
    CHECK(bus.syncWriteRegister(0,1,ids,data,std::numeric_limits<std::size_t>::max())==Result::BUFFER_ERROR);
    CHECK(bus.syncWriteRegister(255,2,ids,data,1)==Result::INVALID_ARGUMENT);
    CHECK(f.writes.empty());
    ids[1]=2; CHECK(bus.syncWriteRegister(0x21,1,ids,data,2)==Result::OK);
    CHECK(f.writes.back()==Bytes({255,255,254,8,0x83,0x21,1,1,1,2,1,0x4f}));
}
static void signed_velocity_limits() {
    Fake f; STServo bus(f);
    CHECK(bus.setVelocity(1,std::numeric_limits<std::int32_t>::min()).ok());
    CHECK(f.writes.back()[6]==255 && f.writes.back()[7]==255);
    CHECK(bus.setVelocity(2,std::numeric_limits<std::int32_t>::max()).ok());
    CHECK(f.writes.back()[6]==255 && f.writes.back()[7]==127);
    CHECK(bus.sts_exchange(2,-500).ok());
    CHECK(f.writes.back()[6]==0xf4 && f.writes.back()[7]==0x81);
}
static void echo_mode_and_real_error_one() {
    Fake f; f.echo=true; f.scripts={status(1)};
    STServo bus(f,10,STServo::EchoMode::LocalEcho);
    auto r=bus.ping(1); CHECK(r.ok() && r.servo_error==0 && r.status_received);
    r=bus.ping(1); CHECK(r.result==Result::ECHO_ONLY && !r.status_received);
    f.echo=false; f.scripts={status(1)};
    CHECK(bus.ping(1).result==Result::ECHO_ERROR);
    bus.setEchoMode(STServo::EchoMode::None); f.scripts={status(1,1)};
    r=bus.ping(1); CHECK(r.ok() && r.servo_error==1 && r.status_received);
}
static void local_echo_no_ack_is_consumed() {
    Fake f; f.echo=true; f.delay=1; f.gap=1;
    STServo bus(f,100,STServo::EchoMode::LocalEcho);
    const std::uint8_t ids[]={1,2}; const std::int32_t speeds[]={100,-100};
    CHECK(bus.syncSetVelocity(ids,speeds,2)==Result::OK);
    CHECK(f.position==f.pending.size() && f.now>=14);
    f.scripts={status(1)};
    const auto r=bus.ping(1); CHECK(r.ok() && r.status_received && !r.servo_error);
}
static void transport_failures() {
    Fake f; STServo bus(f);
    f.clear_ok=false; CHECK(bus.ping(1).result==Result::TRANSPORT_ERROR); CHECK(f.writes.empty());
    f.clear_ok=true; f.write_ok=false; CHECK(bus.ping(1).result==Result::TRANSPORT_ERROR);
    f.write_ok=true; f.read_error=true; CHECK(bus.ping(1).result==Result::TRANSPORT_ERROR);
}
static void invalid_arguments() {
    Fake f; STServo bus(f); std::uint8_t byte=0;
    CHECK(bus.ping(254).result==Result::INVALID_ARGUMENT);
    CHECK(bus.ping(255).result==Result::INVALID_ARGUMENT);
    CHECK(bus.readRegister(1,0,nullptr,1).result==Result::INVALID_ARGUMENT);
    CHECK(bus.readRegister(1,255,&byte,2).result==Result::INVALID_ARGUMENT);
    CHECK(bus.readRegister(1,0,&byte,0).result==Result::INVALID_ARGUMENT);
    CHECK(bus.readRegister(1,0,&byte,59).result==Result::BUFFER_ERROR);
    CHECK(bus.writeRegister(1,0,&byte,58).result==Result::BUFFER_ERROR);
    CHECK(bus.writeRegister(1,0,nullptr,1).result==Result::INVALID_ARGUMENT);
    CHECK(f.writes.empty());
    CHECK(!bus.setTimeoutMs(0)); CHECK(!bus.setTimeoutMs(0x80000000U)); CHECK(bus.timeoutMs()==10);
    STServo invalid(f,0); CHECK(invalid.ping(1).result==Result::INVALID_ARGUMENT);
}
static void packet_limits() {
    Fake f; STServo bus(f); Bytes payload(STServo::MaxReadSize,0x5a);
    f.scripts={status(253,0,payload)}; std::uint8_t output[STServo::MaxReadSize]={};
    auto r=bus.readRegister(253,0,output,sizeof(output));
    CHECK(r.ok() && r.parameter_count==sizeof(output));
    CHECK(Bytes(output,output+sizeof(output))==payload);
    std::uint8_t data[STServo::MaxWriteSize]={};
    CHECK(bus.writeRegister(0,0,data,sizeof(data)).ok());
    CHECK(f.writes.back().size()==STServo::MaxPacketSize);
}
static void stale_input_is_discarded() {
    Fake f; f.pending=status(1,8); f.scripts={status(1)}; STServo bus(f);
    const auto r=bus.ping(1); CHECK(r.ok() && r.servo_error==0 && f.clears==1);
}
static void total_deadline_includes_write() {
    Fake f; f.write_time=5; f.scripts={status(1)}; STServo bus(f,5);
    CHECK(bus.ping(1).result==Result::TIMEOUT);
}
static void checksum_helpers() {
    std::uint8_t frame[]={255,255,2,2,1,0}; STServo::genCheckSum(frame,sizeof(frame));
    CHECK(frame[5]==250);
    CHECK(STServo::receiveCheckSum(frame+2,4)==0);
    frame[5]^=1; CHECK(STServo::receiveCheckSum(frame+2,4)==0x10);
    CHECK(STServo::receiveCheckSum(nullptr,4)==0x23);
    STServo::genCheckSum(nullptr,0);
}
int main() {
    struct Test { const char* name; void(*run)(); };
    const Test tests[]={
        {"multi ID result isolation",multi_id_results}, {"noise/other ID/fragmentation",other_id_noise_and_fragments},
        {"checksum/length recovery",checksum_and_length_recovery}, {"timeout/tick wrap",timeout_wrap_and_wrong_id},
        {"read/error output commit",read_commit_and_servo_error}, {"write/ACK policy",writes_and_ack_policy},
        {"control helpers",control_register_helpers}, {"position targets",position_targets}, {"ID change/readback",id_change_validation_and_readback},
        {"sync velocity wire packet",sync_velocity_golden_packet}, {"sync validation",sync_validation},
        {"signed velocity limits",signed_velocity_limits}, {"echo/real servo ERROR=1",echo_mode_and_real_error_one},
        {"no-ACK local echo drain",local_echo_no_ack_is_consumed}, {"transport failures",transport_failures}, {"argument boundaries",invalid_arguments},
        {"maximum packets",packet_limits}, {"stale RX",stale_input_is_discarded},
        {"whole-operation deadline",total_deadline_includes_write}, {"checksum helpers",checksum_helpers}
    };
    int failures=0;
    for (const auto& test:tests) {
        try { test.run(); std::printf("PASS %s\n",test.name); }
        catch(const std::exception& error) { ++failures; std::fprintf(stderr,"FAIL %s: %s\n",test.name,error.what()); }
    }
    std::printf("%zu protocol test groups, %d failures\n",sizeof(tests)/sizeof(tests[0]),failures);
    return failures?1:0;
}
