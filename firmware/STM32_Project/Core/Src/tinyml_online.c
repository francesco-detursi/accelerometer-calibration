#include "tinyml_online.h"

#include "network.h"
#include "user_init.h"

STAI_NETWORK_CONTEXT_DECLARE(network_context, STAI_NETWORK_CONTEXT_SIZE)

STAI_ALIGNED(STAI_NETWORK_ACTIVATION_1_ALIGNMENT)
static uint8_t network_activations[STAI_NETWORK_ACTIVATION_1_SIZE_BYTES];

STAI_ALIGNED(STAI_NETWORK_IN_1_ALIGNMENT)
static float network_input[3];

STAI_ALIGNED(STAI_NETWORK_OUT_1_ALIGNMENT)
static float network_output[3];

static const float input_mean[3] =
{
    -1.551330871582e+02f,
    1.982502441406e+02f,
    6.282903289795e+01f
};

static const float input_std[3] =
{
    3.134229736328e+03f,
    3.445519042969e+03f,
    3.746150878906e+03f
};

static const float calibration_matrix[3][3] =
{
    {0.0001670409153043474f, 3.8926801119692142e-07f, -4.1935540379913426e-07f},
    {0.0f, 0.00016717811351980427f, 2.935471171398182e-07f},
    {0.0f, 0.0f, 0.00016708300495685963f}
};

static const float calibration_bias[3] =
{
    0.27588688009800849f,
    21.458496340481911f,
    43.41020555601407f
};

static float absf_local(float x)
{
    return (x >= 0.0f) ? x : -x;
}

stai_return_code TinyML_Online_Init(void)
{
    stai_return_code status;

    status = stai_runtime_init();
    if (status != STAI_SUCCESS)
        return status;

    status = user_stai_network_init(network_context);
    if (status != STAI_SUCCESS)
        return status;

    stai_ptr activations[STAI_NETWORK_ACTIVATIONS_NUM] =
    {
        (stai_ptr)network_activations
    };

    status = stai_network_set_activations(
        network_context,
        activations,
        STAI_NETWORK_ACTIVATIONS_NUM);
    if (status != STAI_SUCCESS)
        return status;

    stai_ptr inputs[STAI_NETWORK_IN_NUM] =
    {
        (stai_ptr)network_input
    };

    status = stai_network_set_inputs(
        network_context,
        inputs,
        STAI_NETWORK_IN_NUM);
    if (status != STAI_SUCCESS)
        return status;

    stai_ptr outputs[STAI_NETWORK_OUT_NUM] =
    {
        (stai_ptr)network_output
    };

    return stai_network_set_outputs(
        network_context,
        outputs,
        STAI_NETWORK_OUT_NUM);
}

stai_return_code TinyML_Online_Run(
    int16_t raw_x,
    int16_t raw_y,
    int16_t raw_z,
    TinyML_OnlineResult_t *result)
{
    if (result == 0)
        return STAI_ERROR_GENERIC;

    result->raw[0] = (float)raw_x;
    result->raw[1] = (float)raw_y;
    result->raw[2] = (float)raw_z;

    for (uint32_t i = 0; i < 3U; ++i)
        network_input[i] = (result->raw[i] - input_mean[i]) / input_std[i];

    float shifted[3];
    for (uint32_t i = 0; i < 3U; ++i)
        shifted[i] = result->raw[i] + calibration_bias[i];

    for (uint32_t i = 0; i < 3U; ++i)
    {
        result->reference[i] =
            calibration_matrix[i][0] * shifted[0] +
            calibration_matrix[i][1] * shifted[1] +
            calibration_matrix[i][2] * shifted[2];
    }

    stai_return_code status = stai_network_run(
        network_context,
        STAI_MODE_SYNC);

    if (status != STAI_SUCCESS)
        return status;

    float sum = 0.0f;
    float maximum = 0.0f;

    for (uint32_t i = 0; i < 3U; ++i)
    {
        result->neural[i] = network_output[i];
        result->error[i] = result->neural[i] - result->reference[i];

        float absolute_error = absf_local(result->error[i]);
        sum += absolute_error;

        if (absolute_error > maximum)
            maximum = absolute_error;
    }

    result->max_abs_error = maximum;
    result->mean_abs_error = sum / 3.0f;

    return STAI_SUCCESS;
}
