#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <fstream>
#include <algorithm>
#include "geometry.h" // Cложение, вычитание, умножение на скаляр, скалярное произведение, присваивание векторов.

// Источник света
struct Light {
	Vec3f position;
	float intensity;

	Light(const Vec3f& position, const float intensity) {
		this->position = position;
		this->intensity = intensity;
	}
};

// Материал
struct Material {
	Vec4f albedo;
	Vec3f diffuse_color;
	float specular_exponent;
	float refractive_index;

	Material() {
		albedo = Vec4f(1, 0, 0, 0);
		diffuse_color = Vec3f();
		specular_exponent = 0;
		refractive_index = 1;
	}

	Material(const float refractive_index, const Vec4f& albedo, const Vec3f& color, const float specular_exponent) {
		this->refractive_index = refractive_index;
		this->albedo = albedo;
		diffuse_color = color;
		this->specular_exponent = specular_exponent;
	}
};

// Сфера
struct Sphere {
	Vec3f sphere_center;
	float radius;
	Material material;

	Sphere(const Vec3f& sphere_center, const float radius, const Material& material) {
		this->sphere_center = sphere_center;
		this->radius = radius;
		this->material = material;
	}

	// Пересекает ли луч сферу
	bool intersection(const Vec3f& viewpoint, const Vec3f& direction, float& t0) const {
		Vec3f v = sphere_center - viewpoint;
		float tca = v * direction;
		float d2 = v * v - tca * tca;
		if (d2 > radius* radius)
			return false;
		float thc = sqrtf(radius * radius - d2);
		t0 = tca - thc;
		float t1 = tca + thc;
		if (t0 < 0)
			t0 = t1;
		if (t0 < 0)
			return false;
		return true;
	}
};

// Отражение
Vec3f reflection(const Vec3f& L, const Vec3f& N) {
	return L - N * 2.f * (L * N); // по закону Фонга
}

// Преломление
Vec3f refraction(const Vec3f& L, const Vec3f& N, const float eta_t, const float eta_i = 1.f) { // по закону Снеллиуса
	float cosi = -std::max(-1.f, std::min(1.f, L * N));

	if (cosi < 0)
		return refraction(L, -N, eta_i, eta_t); // если луч выходи изнутри объекта, то меняем среды

	float eta = eta_i / eta_t;
	float k = 1 - eta * eta * (1 - cosi * cosi);

	return k < 0 ? Vec3f(1, 0, 0) : L * eta + N * (eta * cosi - sqrtf(k));
}

// Пересечение луча с объектам сцены
bool scene_intersection(const Vec3f& origin, const Vec3f& direction, const std::vector<Sphere>& spheres, Vec3f& hit, Vec3f& N, Material& material) {
	float spheres_dist = std::numeric_limits<float>::max();
	for (size_t i = 0; i < spheres.size(); i++) {
		float dist_i;
		if (spheres[i].intersection(origin, direction, dist_i) && dist_i < spheres_dist) {
			spheres_dist = dist_i;
			hit = origin + direction * dist_i;
			N = (hit - spheres[i].sphere_center).normalize();
			material = spheres[i].material;
		}
	}

	// Отрисовка пола
	float floor = std::numeric_limits<float>::max();
	if (fabs(direction.y) > 1e-3) {
		float d = -(origin.y + 5) / direction.y;
		Vec3f pt = origin + direction * d;
		if (d > 0 && fabs(pt.x) < 10 && pt.z<-10 && pt.z>-30 && d < spheres_dist) {
			floor = d;
			hit = pt;
			N = Vec3f(0, 1, 0);
			material.diffuse_color = (int(.5 * hit.x + 1000) + int(.5 * hit.z)) & 1 ? Vec3f(.3, .3, .3) : Vec3f(.3, .2, .1);
		}
	}
	return std::min(spheres_dist, floor) < 1000;
}

// Пуск луча от точки смотрителя в заданном направлении
Vec3f ray_casting(const Vec3f& viewpoint, const Vec3f& direction, const std::vector<Sphere>& spheres, const std::vector<Light>& lights, size_t depth = 0) {
	Vec3f point, N;
	Material material;

	if (depth > 5 || !scene_intersection(viewpoint, direction, spheres, point, N, material)) {
		return Vec3f(0.5, 0.2, 0.4); //цвет пикселя, если луч ничего не пересек
	}

	Vec3f reflect_direction = reflection(direction, N).normalize();
	Vec3f refract_direction = refraction(direction, N, material.refractive_index).normalize();

	Vec3f reflect_point = reflect_direction * N < 0 ? point - N * 1e-3 : point + N * 1e-3; // сдвиг точки в направлении нормали, потому что
	Vec3f refract_point = refract_direction * N < 0 ? point - N * 1e-3 : point + N * 1e-3; // точка лежит на поверхности объекта, и любой луч из этой точки будет пересекать сцену

	Vec3f reflect_color = ray_casting(reflect_point, reflect_direction, spheres, lights, depth + 1);
	Vec3f refract_color = ray_casting(refract_point, refract_direction, spheres, lights, depth + 1);

	float diffuse_light_intensity = 0, specular_light_intensity = 0;

	for (size_t i = 0; i < lights.size(); i++) {
		Vec3f light_direction = (lights[i].position - point).normalize();
		float light_distance = (lights[i].position - point).norm();

		Vec3f shadow_point = light_direction * N < 0 ? point - N * 1e-3 : point + N * 1e-3; // лежит ли точка в тени от источников света
		Vec3f shadow_pt, shadow_N;
		Material tmpmaterial;

		if (scene_intersection(shadow_point, light_direction, spheres, shadow_pt, shadow_N, tmpmaterial) && (shadow_pt - shadow_point).norm() < light_distance)
			continue;

		diffuse_light_intensity += lights[i].intensity * std::max(0.f, light_direction * N);
		specular_light_intensity += powf(std::max(0.f, -reflection(-light_direction, N) * direction), material.specular_exponent) * lights[i].intensity;
	}

	return material.diffuse_color * diffuse_light_intensity * material.albedo[0]
		+ Vec3f(1., 1., 1.) * specular_light_intensity * material.albedo[1]
		+ reflect_color * material.albedo[2]
		+ refract_color * material.albedo[3];
}

// Рендер сцены
void render(const std::vector<Sphere>& spheres, const std::vector<Light>& lights) {
	const int width = 3840;
	const int height = 2160;
	const float fov = M_PI / 3.;
	std::vector<Vec3f> framebuffer(width * height);

#pragma omp parallel for
	for (size_t j = 0; j < height; j++) {
		for (size_t i = 0; i < width; i++) {

			float x = (i + 0.5) - width / 2.;
			float y = -(j + 0.5) + height / 2.;
			float z = -height / (2. * tan(fov / 2.));
			framebuffer[i + j * width] = ray_casting(Vec3f(0, 0, 0), Vec3f(x, y, z).normalize(), spheres, lights);

		}
	}

	std::ofstream ofs;
	ofs.open("./result.ppm", std::ios::binary);
	ofs << "P6\n" << width << " " << height << "\n255\n";
	for (size_t i = 0; i < height * width; ++i) {
		Vec3f& c = framebuffer[i];
		float max = std::max(c[0], std::max(c[1], c[2]));
		if (max > 1) c = c * (1. / max);
		for (size_t j = 0; j < 3; j++) {
			ofs << (char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
		}
	}
	ofs.close();
}

int main() {
	Material ivory(1.0, Vec4f(0.6, 0.3, 0.1, 0.0), Vec3f(0.4, 0.4, 0.3), 50.);
	Material fuchsia(1.0, Vec4f(0.6, 0.3, 0.1, 0.0), Vec3f(0.4, 0.1, 0.7), 50.);
	Material red_rubber(1.0, Vec4f(0.9, 0.1, 0.0, 0.0), Vec3f(0.3, 0.1, 0.1), 10.);
	Material mirror(1.0, Vec4f(0.0, 10.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425.);
	Material glass(1.5, Vec4f(0.0, 0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8), 125.);

	std::vector<Sphere> spheres;
	spheres.push_back(Sphere(Vec3f(-3, -1, -16), 2, ivory));
	spheres.push_back(Sphere(Vec3f(6, -4, -12), 1, fuchsia));
	spheres.push_back(Sphere(Vec3f(-7, -1, -12), 2, fuchsia));
	spheres.push_back(Sphere(Vec3f(-1.0, -2.5, -12), 2, glass));
	spheres.push_back(Sphere(Vec3f(1.5, -0.5, -18), 3, red_rubber));
	spheres.push_back(Sphere(Vec3f(7, 5, -18), 4, mirror));

	std::vector<Light> lights;
	lights.push_back(Light(Vec3f(-17, 20, 20), 1.5));
	lights.push_back(Light(Vec3f(31, 45, -24), 1.8));
	lights.push_back(Light(Vec3f(32, 20, 30), 1.7));

	render(spheres, lights);
}