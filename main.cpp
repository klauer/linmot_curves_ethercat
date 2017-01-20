#include <cstdio>
#include <cstdint>
#include <cstddef>
#include <cassert>
#include <cstring>
#include <string>
#include <vector>


const double MAX_CURVE_TIME = 42949.0; // 2**32/1e5;


enum LmObjectType {
    LM_OT_POSITION_VS_TIME=0x0003,
    LM_OT_ENCODER_VS_POSITION=0x0103,
};


enum LmUUID {
    LM_UUID_POSITION=0x0005,
    LM_UUID_TIME=0x001A,
    LM_UUID_ENCODER=0x001B,
};


enum LmConfigModuleCmd {
    LM_CMD_NOOP=0x0000,
    LM_CMD_PREPARE_CONFIG_MODULE=0x000F,
    LM_CMD_SAVE_TO_FLASH=0x4001,
    LM_CMD_DELETE_ALL_CURVES=0x4101,
    LM_CMD_RESTART_DRIVE=0x3000,
    LM_CMD_STOP_MC=0x3500,
    LM_CMD_START_MC=0x3600
};


enum LmCommandHeader {
    LM_HDR_TIME_CURVE_SCALED=0x0440,
    LM_HDR_TIME_CURVE_TOTAL=0x0450,
    LM_HDR_INVALIDATE_CURVE=0x0500
};


typedef struct __attribute__((packed)) {
    uint16_t curve_id;
    int32_t curve_offset;
    int32_t time;
    int16_t amplitude_scale;
} LmTimeCurveTotal;


const uint16_t LM_CI_DATA_OFFSET = 70;
const uint16_t LM_CI_STRING_LENGTH = 22;
const int LM_CI_NUM_WIZARD_PARAMS = 7;
const double LM_CI_XLENGTH_SCALE = 1.0e5;


struct __attribute__((packed)) LmCurveInfo {
    uint16_t data_offset;
    uint16_t object_type;
    uint16_t num_setpoints;
    uint16_t data_type_size;
    unsigned char name[LM_CI_STRING_LENGTH];
    uint16_t curve_id;
    uint32_t x_length;
    uint16_t x_dim_uuid;
    uint16_t y_dim_uuid;
    uint16_t wizard_type;
    uint32_t wizard_params[LM_CI_NUM_WIZARD_PARAMS];

    // pad to 72 bytes for easy sending over the wire
    uint16_t __padding__;

    LmCurveInfo() : data_offset(LM_CI_DATA_OFFSET), data_type_size(4),
                    __padding__(0), wizard_type(0) {
        for (int i=0; i < LM_CI_NUM_WIZARD_PARAMS; i++) {
            wizard_params[i] = 0;
        }
    }

    bool set_name(const char* new_name) {
        if (!new_name || strlen((char *)new_name) >= LM_CI_STRING_LENGTH)
            return false;
        strcpy((char*)name, new_name);
        return true;
    }

    bool set_name(std::string& new_name) {
        return set_name(new_name.c_str());
    }

    void dump(FILE *fp=stdout) {
        char sname[LM_CI_STRING_LENGTH + 1];
        sname[LM_CI_STRING_LENGTH] = 0;
        memcpy(sname, name, LM_CI_STRING_LENGTH);

        fprintf(fp, "--------------------\n");
        fprintf(fp, "data_offset:      %u\n", data_offset);
        fprintf(fp, "object_type:      %u\n", object_type);
        fprintf(fp, "num_setpoints:    %u\n", num_setpoints);
        fprintf(fp, "data_type_size:   %u\n", data_type_size);
        fprintf(fp, "name:             %s\n", sname);
        fprintf(fp, "curve_id:         %u\n", curve_id);
        fprintf(fp, "x_length:         %u\n", x_length);
        fprintf(fp, "x_dim_uuid:       0x%x\n", x_dim_uuid);
        fprintf(fp, "y_dim_uuid:       0x%x\n", y_dim_uuid);
        fprintf(fp, "wizard_type:      %u\n", wizard_type);

        for (int i=0; i < LM_CI_NUM_WIZARD_PARAMS; i++) {
            fprintf(fp, "wizard_params[%d]: %u\n", i, wizard_params[i]);
        }
    }
};

typedef struct LmCurveInfo LmCurveInfo;


bool new_position_time_curve(struct LmCurveInfo &ci, double time_sec,
                             uint16_t num_setpoints, std::string& name,
                             uint16_t curve_id)
{
    ci.data_offset = (uint16_t)LM_CI_DATA_OFFSET;
    ci.object_type = (uint16_t)LM_OT_POSITION_VS_TIME;

    if (time_sec <= 0.0 || time_sec >= MAX_CURVE_TIME)
        return false;

    ci.num_setpoints = num_setpoints;
    ci.data_type_size = 4;
    ci.set_name(name);
    ci.curve_id = curve_id;
    // x-length is in units of 10us
    ci.x_length = (uint32_t)(time_sec * LM_CI_XLENGTH_SCALE);
    ci.x_dim_uuid = LM_UUID_TIME;
    ci.y_dim_uuid = LM_UUID_POSITION;
    ci.wizard_type = 0;
    for (int i=0; i < LM_CI_NUM_WIZARD_PARAMS; i++) {
        ci.wizard_params[i] = 0;
    }
    ci.__padding__ = 0;
    return true;
}

class LmCurve {
public:
    std::string name;
    uint16_t curve_id;
    std::vector<double> setpoints;

    LmCurve()
    {
    }

    LmCurve(std::string& _name, uint16_t _curve_id) :
        name(_name), curve_id(_curve_id)
    {
    }

};

class LmPositionTimeCurve : public LmCurve {
public:
    double dt;

    LmPositionTimeCurve() : LmCurve()
    {
    }

    LmPositionTimeCurve(std::string& _name, uint16_t _curve_id, double _dt) :
        LmCurve(_name, _curve_id), dt(_dt)
    {
    }

    struct LmCurveInfo *get_curve_info() {
        LmCurveInfo *ci = new LmCurveInfo;
        int num_setpoints = setpoints.size();
        if (new_position_time_curve((LmCurveInfo&)ci, get_total_seconds(),
                                    num_setpoints, name, curve_id)) {
            return ci;
        } else {
            delete ci;
            return NULL;
        }
    }

    void set_curve_info(LmCurveInfo &ci) {
        char sname[LM_CI_STRING_LENGTH + 1];
        sname[LM_CI_STRING_LENGTH] = 0;
        memcpy((void*)sname, ci.name, LM_CI_STRING_LENGTH);

        name = (const char*)sname;
        curve_id = ci.curve_id;
        if (ci.x_length > 0 && ci.num_setpoints > 0) {
            dt = (double)ci.x_length / ((double)ci.num_setpoints * LM_CI_XLENGTH_SCALE);
        } else {
            dt = 0.0;
        }
    }

    const double get_total_seconds() {
        int num_setpoints = setpoints.size();
        return num_setpoints * dt;
    }

};



int main(void)
{
    assert(__LITTLE_ENDIAN__);
    assert((offsetof(LmCurveInfo, data_offset) == 0));
    assert((offsetof(LmCurveInfo, wizard_params[6]) == 66));
    assert(sizeof(LmCurveInfo) == 72);

    assert(sizeof(LmTimeCurveTotal) == 12);

    LmCurveInfo ci;

    uint32_t info_array[] = {
        196678, 262645, 1131898190, 1702261365, 0, 0, 0, 65536, 100000, 327706,
        -2036333311, 666894337, -1591738359, 7, 0, 0, 0, 666894336
    };

    printf("%lu %lu\n", sizeof(info_array), sizeof(ci));
    assert(sizeof(info_array) >= sizeof(ci));
    memcpy(&ci, info_array, sizeof(ci));

    ci.dump();

    LmCurveInfo ci2;
    std::string curve_name("NewCurve");
    new_position_time_curve(ci2, 1.0f, 501, curve_name, 1);
    ci2.dump();

    LmPositionTimeCurve c(curve_name, 1, 0.01);
    c.setpoints.push_back(0.0);
    c.setpoints.push_back(1.0);
    c.setpoints.push_back(2.0);
    LmCurveInfo *ci3 = c.get_curve_info();
    ci3->dump();

    LmPositionTimeCurve c2;
    c2.set_curve_info(*ci3);

    delete ci3;
    return 0;
}
