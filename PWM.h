#include <sys/types.h>
#include <stdint.h>

void pca_init(int *pca_success);
void pwm_set(uint8_t port, uint16_t duration);
void pwm_set_all(uint16_t duration_1, uint16_t duration_2, uint16_t duration_3, uint16_t duration_4);