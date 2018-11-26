/**
 * @brief   Convert the adc value into the current speed
 *
 * @param[in] adc_value Converted digital adc value
 * @return number Integer value representing the current speed
 */
int get_speed(uint16_t adcValue);
/**
 * @brief   Identifies wether speed is above limit ot not.
 *
 * @param[in] speed    Integer to indicate speed value
 * @param[in] limit    Integer to indicate limit to compair
 * @return  Null if speed is NOT above limit and 1 if speed is above limit
 */
int is_speed_above_limit(int speed, int limit);