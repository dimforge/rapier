#include <rapier.hpp>

int main() {
    auto world = rapier::make_world();
    rapier::check(RAPIER_FN(Step)(world.get(), nullptr, nullptr));
}
