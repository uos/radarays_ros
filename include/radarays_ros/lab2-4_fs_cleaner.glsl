#version 440

out vec4 frag_colour;

#define M_PI 3.14159
  
in vec4 normal;
in vec4 position;
uniform int light_count;
uniform vec4 light_position[4];
uniform vec4 light_colour[4];


//-------------------------------------------------------------------------//
// Lambertian BRDF
vec3 lambertian_brdf( vec3 in_direction, vec3 out_direction, vec3 normal )
{
  vec3 Cd =
    {1,1,1};
    //{0.8, 0.8, 0.4, 1.0};
  return Cd *
    vec3(1.0/M_PI);
}

//-------------------------------------------------------------------------//
// Cook-Torrance and related microfacet BRDFs
float Dfunc( float roughness, float n_dot_h )
{
	// The original paper of Cook and Torrance says:
	// float D = (1/(m*m * pow( cos(alpha), 4.0))) * exp (-pow(tan(alpha)/m, 2.0));
	// with alpha = the angle between H and N

	// The book Real-Time Rendering 4 (eq 9.35) says:
	float D =
		max(0.0, n_dot_h) // This is == Xi+(n,m) in the book
		/ (M_PI * roughness*roughness * pow(n_dot_h , 4.0))
		* exp(
		      (n_dot_h*n_dot_h - 1)
		      /
		      (roughness*roughness * n_dot_h*n_dot_h)
		      )
		;
	// The book says dot(n,m) but that is the same as dot(n,h) since
	// only micronormals m that are exactly = h contribute.
	// The term in the exponent is in fact equivalent to the tan term in
	// cookpaper.pdf, since tan^2(theta) == ((1-theta^2)/theta^2)
	// See http://www.codinglabs.net/article_physically_based_rendering_cook_torrance.aspx
	return D;
}


float Dfunc_GGX( float roughness, float n_dot_h )
{
	// This is the GGX distribution function (eq 9.41) from Real-Time Rendering 4
	float D =
		(max(0.0,n_dot_h) * roughness*roughness)
		/
		( M_PI * pow((1 + n_dot_h*n_dot_h * (roughness*roughness-1)), 2) );
	return D;
}

float Gfunc( float n_dot_h, float o_dot_h,
             float n_dot_o, float n_dot_i )
{
	float G1 = 2 * (n_dot_h * n_dot_o) / (o_dot_h);
	float G2 = 2 * (n_dot_h * n_dot_i) / (o_dot_h);
	float G = min( G1, G2 );
	G = min( 1, G );
	return G;
}

// Helper for cook_torrance_brdf: the Fresnel term (eq 9.16)
vec3 Fresnel(vec3 h, vec3 in_direction, vec3 F0 )
{
	return F0 + (1 - F0) * pow(1 - dot( h, in_direction ), 5.0 );
}

// Using eq (9.34) from RTR4
vec3 cook_torrance_brdf( vec3 in_direction, vec3 out_direction, vec3 normal )
{
	float k_L = 0.2;
	vec3 rho =  // diffuse colour
	//{1, 1, 1};
	//{0.8, 0.5, 0.3};
	//{0.8, 0.8, 0.5};
	//{0.955,0.638,0.538}; // copper, according to Table 9.2
		{1, 0.782, 0.344}; // gold, according to Table 9.2
	// tab 9.2 -> F0 values
	// why is rho == F0, and F0


	
	float k_g = 1 - k_L;
	vec3 k_s =
		//{1, 1, 1};  // pure white "specular" term = non-metallic
		//vec3(0.04); // stone/plastic, according to Table 9.2
		{1, 0.782, 0.344};          // same as diffuse colour = metallic
	
	float roughness = .3; 

	// specular term

	// // black plastic dice
	// roughness = .1;
	// k_L = 0.7;
	// k_g = 1-k_L;
	// rho = vec3(.02,.02,.02);
	// k_s = vec3(1);

	// red velvet
	roughness = 1.5;
	k_L = 0.25;
	k_g = 1-k_L;
	rho = vec3(.7,.1,.1);
	k_s = vec3(1);
	
	vec3 result;

	vec3 h = normalize(out_direction + in_direction); // half-vector
	float n_dot_h = max(0, dot( normal, h ) );
	float n_dot_o = max(0, dot( normal, out_direction ) );
	float n_dot_i = max(0, dot( normal, in_direction ) );
	float o_dot_h = max(0, dot( out_direction, h ) );

	vec3 F = Fresnel(h, in_direction, k_s );
	float D = Dfunc( roughness, n_dot_h ); // Use `Dfunc` or `Dfunc_GGX`
	float G = Gfunc( n_dot_h, o_dot_h, n_dot_o, n_dot_i );
	
	result =
		k_L*rho/M_PI +
		k_g*1*( (F*G*D) / ( 4*n_dot_i*n_dot_o ) );

	result = clamp(result,0.0,1.0);
	return result;
}



//-------------------------------------------------------------------------//
// Blinn-Phong BRDF

vec3 blinn_phong_brdf( vec3 in_direction, vec3 out_direction, vec3 normal )
{
	float k_L = 0.9; // diffuse amount
	vec3 rho =  // diffuse colour
	//{1, 1, 1};
		{0.8, 0.5, 0.3};
	float k_g = 0.1; // glossy amount
	vec3 k_s =
		{1, 1, 1}; // pure white "specular" term = non-metallic
	//rho;
	float s = 100;
  
	vec3 w_h = normalize( in_direction.xyz + out_direction.xyz );

	vec3 result = k_L * rho / M_PI + k_g * k_s * (8 + s) / (8 * M_PI) * pow( max(0,dot( normal.xyz, w_h )), s );

	return result;
}


//-------------------------------------------------------------------------//
// Oren-Nayar BRDF

// Helper function for Oren-Nayar
float compute_azimuth2( vec3 in_direction )
{
	vec3 wo = normalize(-position.xyz); //out = view dir
	vec3 x_dir = normalize( wo - dot(wo,normal.xyz) * normal.xyz ); // project down 
	vec3 y_dir = cross( normal.xyz, x_dir ); // compute orthogonal basis
	vec3 wi = normalize(in_direction.xyz); // light in direction
	vec3 B = wi - dot(wi,normal.xyz) * normal.xyz; // project down
	float Bx = dot( x_dir, B ); // x coordinate of projected in vector, with x axis pointing towards out dir
	float By = dot( y_dir, B ); // y coordinate
	return atan(By,Bx); // angle: phi_out - phi_in
}

vec3 oren_nayar_brdf( vec3 in_direction, vec3 out_direction, vec3 normal )
{
	float theta_in = acos(dot(in_direction,normal));
	float theta_out = acos(dot(out_direction,normal));

	float phi = compute_azimuth2( in_direction );
	float phi_term = 	max(0,cos(phi));
    
	float alpha = max(theta_in, theta_out);
	float beta = min(theta_in, theta_out);
	float sigma2 = 0.5; // angle variance = surface roughness. Gets a little weird if this is >>0.5. That's consistent with O-N's observation that a Gaussian model is not fitting when the variance is too large.
	float rho = 1; // rho = albedo // This should be the rgb colour.
	float A = 1.0 - 0.5 * (sigma2 / (sigma2 + 0.33));
	float B = 0.45 * sigma2 / (sigma2 + 0.09);

	float L = rho/M_PI * (A+B*phi_term*sin(alpha)*tan(beta));
  
	vec3 result;
	result = clamp(vec3(L),0.0,1.0);
	return result;
}


//-------------------------------------------------------------------------//
// main

void main () {
	vec3 colour = vec3(0.0);

	// Loop over light sources
	for (int l = 0; l < light_count; ++l )
  {
	  float cosine = dot( normalize(normal), normalize(light_position[l] - position) );
	  // Compute rendering equation
	  if (cosine > 0)
	  {
		  colour +=
			  // incoming irradiance:
			  3.14*
			  light_colour[l].xyz *
			  // BRDF:
			  //blinn_phong_brdf
			  //cook_torrance_brdf
        oren_nayar_brdf
        //lambertian_brdf
        ( normalize(light_position[l].xyz - position.xyz),
          normalize( - position.xyz),
          normalize(normal.xyz) )
        *
        cosine;
    }
  }
  frag_colour = vec4(colour,1.0);
}
