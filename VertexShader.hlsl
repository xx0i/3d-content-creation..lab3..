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
    matrix padding;
};
// TODO: Part 3b
// TODO: Part 3f
// TODO: Part 3g
float4 main(VERTEX input) : SV_POSITION
{
	// TODO: Part 2i
   // input.pos = mul(worldMatrix, input.pos);
	// TODO: Part 3b
	// TODO: Part 3g
	return input.pos;
}