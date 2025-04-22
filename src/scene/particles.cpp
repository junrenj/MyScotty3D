#include "particles.h"


bool Particles::Particle::update(const PT::Aggregate &scene, Vec3 const &gravity, const float radius, const float dt) {

	//A4T4: particle update
	float time = dt;
	// Compute the trajectory of this particle for the next dt seconds.
	while (time > 0.0f)
	{
		float speed = velocity.norm();
		Vec3 dir = velocity / speed;
		
		// (1) Build a ray representing the particle's path as if it travelled at constant velocity.
		Ray ray = Ray(position, dir, Vec2(0.0f, std::numeric_limits<float>::infinity()), 0);
		// (2) Intersect the ray with the scene and account for collisions. Be careful when placing
		// collision points using the particle radius. Move the particle to its next position.
		PT::Trace trace = scene.hit(ray);
		// case 1 : didn't hit
		if(!trace.hit)
		{
			position += velocity * time;
			velocity += gravity * time;
			break;
		}
		else
		{
			// case 2 : didn't hit
			if(trace.distance > speed * time)
			{
				position += velocity * time;
				velocity += gravity * time;
				break;
			}
			Vec3 normal = trace.normal.unit();
			float cos_theta = fabs(dot(dir.unit(), -normal));
			float traveDistance = trace.distance - radius / cos_theta;
			// case 2 : particle is inside the surface
			if(traveDistance <= 0.0f)
			{
				velocity = velocity - 2 * normal * dot(normal, velocity);
				continue;
			}
			else
			{
				// case 3 : successfully hit
				float timeConsume = traveDistance / speed;
				timeConsume = timeConsume > time ? time : timeConsume;
				position += velocity * timeConsume;
				velocity += gravity * timeConsume;
	
				velocity = velocity - 2 * normal * dot(normal, velocity);
	
				time -= timeConsume;
			}
		}
	}
	// (5) Decrease the particle's age and return 'false' if it should be removed.
	age -= dt;
	return age > 0.0f;
}

void Particles::advance(const PT::Aggregate& scene, const Mat4& to_world, float dt) {

	if(step_size < EPS_F) return;

	step_accum += dt;

	while(step_accum > step_size) {
		step(scene, to_world);
		step_accum -= step_size;
	}
}

void Particles::step(const PT::Aggregate& scene, const Mat4& to_world) {

	std::vector<Particle> next;
	next.reserve(particles.size());

	for(Particle& p : particles) {
		if(p.update(scene, gravity, radius, step_size)) {
			next.emplace_back(p);
		}
	}

	if(rate > 0.0f) {

		//helpful when emitting particles:
		float cos = std::cos(Radians(spread_angle) / 2.0f);

		//will emit particle i when i == time * rate
		//(i.e., will emit particle when time * rate hits an integer value.)
		//so need to figure out all integers in [current_step, current_step+1) * step_size * rate
		//compute the range:
		double begin_t = current_step * double(step_size) * double(rate);
		double end_t = (current_step + 1) * double(step_size) * double(rate);

		uint64_t begin_i = uint64_t(std::max(0.0, std::ceil(begin_t)));
		uint64_t end_i = uint64_t(std::max(0.0, std::ceil(end_t)));

		//iterate all integers in [begin, end):
		for (uint64_t i = begin_i; i < end_i; ++i) {
			//spawn particle 'i':

			float y = lerp(cos, 1.0f, rng.unit());
			float t = 2 * PI_F * rng.unit();
			float d = std::sqrt(1.0f - y * y);
			Vec3 dir = initial_velocity * Vec3(d * std::cos(t), y, d * std::sin(t));

			Particle p;
			p.position = to_world * Vec3(0.0f, 0.0f, 0.0f);
			p.velocity = to_world.rotate(dir);
			p.age = lifetime; //NOTE: could adjust lifetime based on index
			next.push_back(p);
		}
	}

	particles = std::move(next);
	current_step += 1;
}

void Particles::reset() {
	particles.clear();
	step_accum = 0.0f;
	current_step = 0;
	rng.seed(seed);
}

bool operator!=(const Particles& a, const Particles& b) {
	return a.gravity != b.gravity
	|| a.radius != b.radius
	|| a.initial_velocity != b.initial_velocity
	|| a.spread_angle != b.spread_angle
	|| a.lifetime != b.lifetime
	|| a.rate != b.rate
	|| a.step_size != b.step_size
	|| a.seed != b.seed;
}
