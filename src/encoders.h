#ifndef ENCODERS_H
#define ENCODERS_H

    #define TOTAL_ENCODER_CHANNELS 2
    #define TOTAL_ENCODER_PINS 2

    typedef struct Encoders {
        byte enc_a_pin_a;
        byte enc_a_pin_b;
        byte enc_b_pin_a;
        byte enc_b_pin_b;
        
        volatile long count_enc_a;
        volatile long count_enc_b;

        long count_enc_a_buf;
        long count_enc_b_buf;

        long prev_count_enc_a;
        long prev_count_enc_b;

        long delta_enc_a;
        long delta_enc_b;
    } Encoders;
    Encoders encoders;

    void IRAM_ATTR encoder_a_callback() {
        bool pin_a = digitalRead(encoders.enc_a_pin_a);
        bool pin_b = digitalRead(encoders.enc_a_pin_b);
        encoders.count_enc_a += (pin_a == pin_b) ? 1 : -1;
    }
    void IRAM_ATTR encoder_b_callback() {
        bool pin_a = digitalRead(encoders.enc_b_pin_a);
        bool pin_b = digitalRead(encoders.enc_b_pin_b);
        encoders.count_enc_b += (pin_a == pin_b) ? 1 : -1;
    }

    void configure_encoders(int enc_a_pin_a, int enc_a_pin_b, int enc_b_pin_a, int enc_b_pin_b) {
        encoders.enc_a_pin_a = enc_a_pin_a;
        encoders.enc_a_pin_b = enc_a_pin_b;
        encoders.enc_b_pin_a = enc_b_pin_a;
        encoders.enc_b_pin_b = enc_b_pin_b;

        pinMode(encoders.enc_a_pin_a, INPUT);
        pinMode(encoders.enc_a_pin_b, INPUT);
        pinMode(encoders.enc_b_pin_a, INPUT);
        pinMode(encoders.enc_b_pin_b, INPUT);

        attachInterrupt(enc_a_pin_a, encoder_a_callback, CHANGE);
        attachInterrupt(enc_b_pin_a, encoder_b_callback, CHANGE);
    }

    long read_enc_count_a() {
        return encoders.count_enc_a_buf;
    }

    long read_enc_count_b() {
        return encoders.count_enc_b_buf;
    }

    void update_enc_values() {
        noInterrupts();
        encoders.count_enc_a_buf = encoders.count_enc_a;
        encoders.count_enc_b_buf = encoders.count_enc_b;
        interrupts();
    
        encoders.delta_enc_a = encoders.count_enc_a_buf - encoders.prev_count_enc_a;
        encoders.delta_enc_b = encoders.count_enc_b_buf - encoders.prev_count_enc_b;
        encoders.prev_count_enc_a = encoders.count_enc_a_buf;
        encoders.prev_count_enc_b = encoders.count_enc_b_buf;
    }

    long read_enc_delta_a() {
        return encoders.delta_enc_a;
    }

    long read_enc_delta_b() {
        return encoders.delta_enc_b;
    }

#endif