//###########################################################################
//
// FILE:   sdfm.c
//
// TITLE:  C28x SDFM Driver
//
//###########################################################################
// $Copyright:  $
//###########################################################################

#include "sdfm.h"

//*****************************************************************************
//
// Defines for filter configurations. Not intended for use by application code.
//
//*****************************************************************************
//
// Get filter oversampling ratio
//
#define SDFM_GET_OSR(C)             ((C) >> 8U)

//
// Maximum acceptable comparator filter oversampling ratio
//
#define SDFM_MAX_COMP_FILTER_OSR    31U

//
// Maximum acceptable data filter oversampling ratio
//
#define SDFM_MAX_DATA_FILTER_OSR    255U

//
// Get the filter type
//
#define SDFM_GET_FILTER_TYPE(C)     ((C) & 0x30U)

//
// Get the filter number
//
#define SDFM_GET_FILTER_NUMBER(C)   ((C) & 0x3U)


//
// Get data shift value
//
#define SDFM_GET_SHIFT_VALUE(C)     (((C) >> 2U) & 0x1FU)

//*****************************************************************************
//
// SDFM_configComparator
//
//*****************************************************************************
void SDFM_configComparator(uint32_t base, uint16_t config1, uint32_t config2)
{
    SDFM_FilterNumber filter;
    uint16_t ratio;
    SDFM_FilterType filterType;

    filter = (SDFM_FilterNumber)(SDFM_GET_FILTER_NUMBER(config1));
    ratio = SDFM_GET_OSR(config1);
    filterType = (SDFM_FilterType)SDFM_GET_FILTER_TYPE(config1);

    //
    // Limit the oversampling ratio
    //
    if(ratio > SDFM_MAX_COMP_FILTER_OSR)
    {
        ratio = SDFM_MAX_COMP_FILTER_OSR;
    }

    //
    // Set the comparator filter type
    //
    SDFM_setComparatorFilterType(base, filter, filterType);

    //
    // Set the comparator filter over sampling ratio
    //
    SDFM_setCompFilterOverSamplingRatio(base, filter, ratio);

    //
    // Set the comparator high threshold value
    //
    SDFM_setCompFilterHighThreshold(base, filter,
                                    SDFM_GET_HIGH_THRESHOLD(config2));

    //
    // Set the comparator low threshold value
    //
    SDFM_setCompFilterLowThreshold(base, filter,
                                   SDFM_GET_LOW_THRESHOLD(config2));

}

//*****************************************************************************
//
// SDFM_configDataFilter
//
//*****************************************************************************
void SDFM_configDataFilter(uint32_t base, uint16_t config1, uint16_t config2)
{
    SDFM_FilterNumber filter;
    uint16_t ratio;
    SDFM_FilterType filterType;

    filter = (SDFM_FilterNumber)(SDFM_GET_FILTER_NUMBER(config1));
    ratio = SDFM_GET_OSR(config1);
    filterType = (SDFM_FilterType)SDFM_GET_FILTER_TYPE(config1);

    //
    // Limit the oversampling ratio
    //
    if(ratio > SDFM_MAX_DATA_FILTER_OSR)
    {
        ratio = SDFM_MAX_DATA_FILTER_OSR;
    }

    //
    // Set the comparator filter type
    //
    SDFM_setFilterType(base, filter, filterType);

    //
    // Set the comparator filter over sampling ratio
    //
    SDFM_setFilterOverSamplingRatio(base, filter, ratio);

    //
    // If filter switch on
    //
    if((config2 & SDFM_FILTER_ENABLE) == SDFM_FILTER_ENABLE)
    {
        SDFM_enableFilter(base, filter);
    }
    else
    {
        SDFM_disableFilter(base, filter);
    }

    //
    // Set output data format
    //
    SDFM_setOutputDataFormat(base, filter,
                             (SDFM_OutputDataFormat)(config2 & 0x1U));

    //
    // Set the shift value if data is in 16-bit 2's complement format
    //
    if((config2 & 0x1U) == (uint16_t)(SDFM_DATA_FORMAT_16_BIT))
    {
        SDFM_setDataShiftValue(base, filter, SDFM_GET_SHIFT_VALUE(config2));
    }
}


