#pragma pack_matrix( row_major )   
// an ultra simple hlsl vertex shader
// TODO: Part 1c
struct VERTEX
{
    float4 pos : POSITION;
};
// TODO: Part 2b
cbuffer shaderVars
{
    matrix worldMatrix;
    // TODO: Part 3b
    matrix viewMatrix;
    matrix perspectiveMatrix;
    // TODO: Part 3f
    // TODO: Part 3g
};

float4 main(VERTEX input) : SV_POSITION
{
	// TODO: Part 2i
    matrix result = mul(viewMatrix, worldMatrix);
	// TODO: Part 3b
   // result = mul(result, perspectiveMatrix);
    input.pos = mul(input.pos, result);
	// TODO: Part 3g
	return input.pos;
}