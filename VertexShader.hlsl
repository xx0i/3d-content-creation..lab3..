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
    matrix worldMatrix[6];
    // TODO: Part 3b
    matrix viewMatrix;
    matrix perspectiveMatrix;
    // TODO: Part 3f
    // TODO: Part 3g
};

float4 main(VERTEX input : POSITION, uint matrixIndex : SV_InstanceID) : SV_POSITION
{
	// TODO: Part 2i
    matrix result = mul(viewMatrix, worldMatrix[matrixIndex]);
	// TODO: Part 3b
    result = mul(result, perspectiveMatrix);
    input.pos = mul(input.pos, result);
	// TODO: Part 3g
	return input.pos;
}