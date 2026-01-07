#ifndef JERRO_DRIVERS_EXPONENTIAL_FILTER_HPP
#define JERRO_DRIVERS_EXPONENTIAL_FILTER_HPP

class ExponentialFilter {
private:
    float alpha = 0.2f;
    float filtered_value = 0;
    bool initialized = false;

public:
    ExponentialFilter(float alpha = 0.2f) : alpha(alpha) {}

    float update(float new_value) {
        if (!initialized) {
            filtered_value = new_value;
            initialized = true;
        } else {
            filtered_value = alpha * new_value + (1.0f - alpha) * filtered_value;
        }
        return filtered_value;
    }

    void reset() {
        filtered_value = 0;
        initialized = false;
    }

    float getValue() const {
        return filtered_value;
    }
};

#endif  // JERRO_DRIVERS_EXPONENTIAL_FILTER_HPP
