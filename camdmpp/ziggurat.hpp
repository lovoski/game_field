/**
 * Source:
 * https://people.sc.fsu.edu/~jburkardt/cpp_src/ziggurat/ziggurat.html
 *
 * ziggurat, a C++ code which generates random variates from the uniform, normal
 * or exponential distributions, by Marsaglia and Tsang. The uniform numbers are
 * generated directly. The ziggurat method is used to compute the normal and
 * exponential values. In the inline version, the underlying generators are
 * implemented "inline", invoking a function call only in exceptional cases.
 * This results in very fast execution. In this implementation, the advantages
 * of inline code are not used. All the routines and inline functions are
 * isolated in a separate file, so that a user invokes them through the familiar
 * library interface. The information on this web page is distributed under the
 * MIT license.
 */
namespace ziggurat {

uint32_t cong_seeded(uint32_t &jcong);
double cpu_time();
uint32_t kiss_seeded(uint32_t &jcong, uint32_t &jsr, uint32_t &w, uint32_t &z);
uint32_t mwc_seeded(uint32_t &w, uint32_t &z);
float r4_exp(uint32_t &jsr, uint32_t ke[256], float fe[256], float we[256]);
void r4_exp_setup(uint32_t ke[256], float fe[256], float we[256]);
float r4_nor(uint32_t &jsr, uint32_t kn[128], float fn[128], float wn[128]);
void r4_nor_setup(uint32_t kn[128], float fn[128], float wn[128]);
float r4_uni(uint32_t &jsr);
uint32_t shr3_seeded(uint32_t &jsr);

}; // namespace ziggurat
