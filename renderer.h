// minimalistic code to draw a single triangle, this is not part of the API.
#include "shaderc/shaderc.h" // needed for compiling shaders at runtime
#ifdef _WIN32 // must use MT platform DLL libraries on windows
#pragma comment(lib, "shaderc_combined.lib") 
#endif

void PrintLabeledDebugString(const char* label, const char* toPrint)
{
	std::cout << label << toPrint << std::endl;

//OutputDebugStringA is a windows-only function 
#if defined WIN32 
	OutputDebugStringA(label);
	OutputDebugStringA(toPrint);
#endif
}

class Renderer
{
	// proxy handles
	GW::SYSTEM::GWindow win;
	GW::GRAPHICS::GVulkanSurface vlk;
	VkRenderPass renderPass;
	GW::CORE::GEventReceiver shutdown;
	
	// what we need at a minimum to draw a triangle
	VkDevice device = nullptr;
	VkPhysicalDevice physicalDevice = nullptr;
	VkBuffer vertexHandle = nullptr;
	VkDeviceMemory vertexData = nullptr;
	VkShaderModule vertexShader = nullptr;
	VkShaderModule fragmentShader = nullptr;
	VkPipeline pipeline = nullptr;
	VkPipelineLayout pipelineLayout = nullptr;

	unsigned int windowWidth, windowHeight;

	// TODO: Part 1c
	struct vertex
	{
		float x, y, z, w;
	};
	// TODO: Part 2a
	GW::MATH::GMATRIXF worldMatrix1 = GW::MATH::GIdentityMatrixF;
	GW::MATH::GMATRIXF worldMatrix2 = GW::MATH::GIdentityMatrixF;
	GW::MATH::GMATRIXF worldMatrix3 = GW::MATH::GIdentityMatrixF;
	GW::MATH::GMATRIXF worldMatrix4 = GW::MATH::GIdentityMatrixF;
	GW::MATH::GMATRIXF worldMatrix5 = GW::MATH::GIdentityMatrixF;
	GW::MATH::GMATRIXF worldMatrix6 = GW::MATH::GIdentityMatrixF;
	GW::MATH::GMatrix interfaceProxy;
	// TODO: Part 2b
	struct shaderVars
	{
		GW::MATH::GMATRIXF worldMatrix[6];
		GW::MATH::GMATRIXF viewMatrix;
		GW::MATH::GMATRIXF perspectiveMatrix;
	};
	shaderVars shaderVarsUniformBuffer{};
	// TODO: Part 3a
	GW::MATH::GMATRIXF viewMatrix = GW::MATH::GIdentityMatrixF;
	// TODO: Part 3f 
	// TODO: Part 2c // TODO: Part 4y
	std::vector<VkBuffer> uniformBufferHandle;
	std::vector<VkDeviceMemory> uniformBufferData;

	// TODO: Part 2e
	VkDescriptorSetLayout descriptorSetLayout = nullptr;

	// TODO: Part 2f
	VkDescriptorPool descriptorPool = nullptr;

	// TODO: Part 2g
	std::vector<VkDescriptorSet> descriptorSets = {};

	// TODO: Part 3c
	GW::MATH::GMATRIXF leftHandedPerspectiveMatrix = GW::MATH::GIdentityMatrixF;
	// TODO: Part 3d
	// TODO: Part 4a
	GW::INPUT::GInput input;
	GW::INPUT::GController controller;
	std::chrono::high_resolution_clock::time_point startTime;

public:
	Renderer(GW::SYSTEM::GWindow _win, GW::GRAPHICS::GVulkanSurface _vlk)
	{
		win = _win;
		vlk = _vlk;

		// TODO: Part 2a
		interfaceProxy.Create();
		startTime = std::chrono::high_resolution_clock::now();
		// TODO: Part 2e
		UpdateWindowDimensions();
		GetHandlesFromSurface();

		//part 2b -> doc said it was supposed to be in the renderer function but that didnt work but it works here ^^
		initializeWorldMatrices();
		// TODO: Part 3a
		initializeViewMatrix();
		shaderVarsUniformBuffer.viewMatrix = viewMatrix;
		// TODO: Part 3c
		initializePerspectiveMatrix();
		shaderVarsUniformBuffer.perspectiveMatrix = leftHandedPerspectiveMatrix;
		// TODO: Part 3d
		// TODO: Part 4a
		input.Create(win);
		controller.Create();

		createDescriptorLayout();
		InitializeGraphics();
		BindShutdownCallback();
	}

	void initializeViewMatrix()
	{
		uint32_t currentImage;
		vlk.GetSwapchainCurrentImage(currentImage);

		GW::MATH::GVECTORF cameraPosition = { 0.25f, -0.125f, -0.25f };
		GW::MATH::GVECTORF targetPosition = { 0.0f, -0.5f, 0.0f };
		GW::MATH::GVECTORF upVector = { 0.0f, 1.0f, 0.0f };
		interfaceProxy.LookAtLHF(cameraPosition, targetPosition, upVector, viewMatrix);
		//shaderVarsUniformBuffer.viewMatrix = viewMatrix;
		//GvkHelper::write_to_buffer(device, uniformBufferData[currentImage], &shaderVarsUniformBuffer, sizeof(shaderVars));
	}

	void initializeWorldMatrices()
	{
		//floor
		GW::MATH::GMATRIXF rotationMatrix = GW::MATH::GIdentityMatrixF;
		GW::MATH::GMATRIXF translationMatrix = GW::MATH::GIdentityMatrixF;
		GW::MATH::GVECTORF floorTranslation = { 0.0f, -0.5f, 0.0f, 1.0f };
		interfaceProxy.RotateXGlobalF(rotationMatrix, G_DEGREE_TO_RADIAN_F(90), rotationMatrix);
		interfaceProxy.TranslateLocalF(translationMatrix, floorTranslation, translationMatrix);
		interfaceProxy.MultiplyMatrixF(rotationMatrix, translationMatrix, worldMatrix1);
		shaderVarsUniformBuffer.worldMatrix[0] = worldMatrix1;

		//ceiling
		rotationMatrix = GW::MATH::GIdentityMatrixF;
		translationMatrix = GW::MATH::GIdentityMatrixF;
		GW::MATH::GVECTORF ceilingTranslation = { 0.0f, 0.5f, 0.0f, 1.0f };
		interfaceProxy.RotateXGlobalF(rotationMatrix, G_DEGREE_TO_RADIAN_F(90), rotationMatrix);
		interfaceProxy.TranslateGlobalF(translationMatrix, ceilingTranslation, translationMatrix);
		interfaceProxy.MultiplyMatrixF(rotationMatrix, translationMatrix, worldMatrix2);
		shaderVarsUniformBuffer.worldMatrix[1] = worldMatrix2;

		//wall 1
		rotationMatrix = GW::MATH::GIdentityMatrixF;
		translationMatrix = GW::MATH::GIdentityMatrixF;
		GW::MATH::GVECTORF wall1Translation = { 0.0f, 0.0f, 0.5f, 1.0f };
		interfaceProxy.TranslateGlobalF(translationMatrix, wall1Translation, translationMatrix);
		interfaceProxy.MultiplyMatrixF(rotationMatrix, translationMatrix, worldMatrix3);
		shaderVarsUniformBuffer.worldMatrix[2] = worldMatrix3;

		//wall 2
		rotationMatrix = GW::MATH::GIdentityMatrixF;
		translationMatrix = GW::MATH::GIdentityMatrixF;
		GW::MATH::GVECTORF wall2Translation = { 0.0f, 0.0f, -0.5f, 1.0f };
		interfaceProxy.TranslateGlobalF(translationMatrix, wall2Translation, translationMatrix);
		interfaceProxy.MultiplyMatrixF(rotationMatrix, translationMatrix, worldMatrix4);
		shaderVarsUniformBuffer.worldMatrix[3] = worldMatrix4;

		//wall 3
		rotationMatrix = GW::MATH::GIdentityMatrixF;
		translationMatrix = GW::MATH::GIdentityMatrixF;
		GW::MATH::GVECTORF wall3Translation = { 0.5f, 0.0f, 0.0f, 1.0f };
		interfaceProxy.RotateYGlobalF(rotationMatrix, G_DEGREE_TO_RADIAN_F(90), rotationMatrix);
		interfaceProxy.TranslateGlobalF(translationMatrix, wall3Translation, translationMatrix);
		interfaceProxy.MultiplyMatrixF(rotationMatrix, translationMatrix, worldMatrix5);
		shaderVarsUniformBuffer.worldMatrix[4] = worldMatrix5;

		//wall 4
		rotationMatrix = GW::MATH::GIdentityMatrixF;
		translationMatrix = GW::MATH::GIdentityMatrixF;
		GW::MATH::GVECTORF wall4Translation = { -0.5f, 0.0f, 0.0f, 1.0f };
		interfaceProxy.RotateYGlobalF(rotationMatrix, G_DEGREE_TO_RADIAN_F(90), rotationMatrix);
		interfaceProxy.TranslateGlobalF(translationMatrix, wall4Translation, translationMatrix);
		interfaceProxy.MultiplyMatrixF(rotationMatrix, translationMatrix, worldMatrix6);
		shaderVarsUniformBuffer.worldMatrix[5] = worldMatrix6;
	}

	void initializePerspectiveMatrix()
	{
		float aspectRatio = 0.0f;
		vlk.GetAspectRatio(aspectRatio);
		interfaceProxy.ProjectionDirectXLHF(G_DEGREE_TO_RADIAN_F(65.0f), aspectRatio, 0.1, 100, leftHandedPerspectiveMatrix);
	}

	void createDescriptorLayout()
	{
		VkDescriptorSetLayoutBinding binding = {};
		binding.binding = 0;
		binding.descriptorCount = 1;
		binding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		binding.pImmutableSamplers = nullptr;
		binding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

		VkDescriptorSetLayoutCreateInfo layoutInfo = {};
		layoutInfo.bindingCount = 1;
		layoutInfo.flags = 0;
		layoutInfo.pBindings = &binding;
		layoutInfo.pNext = nullptr;
		layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;

		vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &descriptorSetLayout);
	}

private:
	void UpdateWindowDimensions()
	{
		win.GetClientWidth(windowWidth);
		win.GetClientHeight(windowHeight);
	}

	void InitializeGraphics()
	{
		InitializeVertexBuffer();
		// TODO: Part 2d
		initializeUniformBuffer();
		// TODO: Part 2f // TODO: Part 4y
		initializeDescriptorPool();
		// TODO: Part 2g // TODO: Part 4y
		initializeDescriptorSets();
		// TODO: Part 2h // TODO: Part 4y
		linkDescriptorSetUniformBuffer();

		CompileShaders();
		InitializeGraphicsPipeline();
	}

	void GetHandlesFromSurface()
	{
		vlk.GetDevice((void**)&device);
		vlk.GetPhysicalDevice((void**)&physicalDevice);
		vlk.GetRenderPass((void**)&renderPass);
	}

	void create2dGrid(vertex verts[], int& index) 
	{
		const float step = 1.0f / 25; //step size

		for (int i = 0; i < 26; i++) //horizontal lines
		{
			float y = -0.5f + i * step;
			verts[index] = { -0.5f, y, 0.0f, 1.0f }; //start of line
			index++;
			verts[index] = { 0.5f, y, 0.0f, 1.0f };  //end of line
			index++;
		}

		for (int i = 0; i < 26; i++) //vertical lines
		{
			float x = -0.5f + i * step;

			verts[index] = { x, -0.5f, 0.0f, 1.0f }; //start of line
			index++;
			verts[index] = { x, 0.5f, 0.0f, 1.0f };  //end of line
			index++;
		}
	}

	void InitializeVertexBuffer()
	{
		// TODO: Part 1b
		// TODO: Part 1c
		// TODO: Part 1d
		vertex verts[104]{};
		int index = 0;
		create2dGrid(verts, index);
		CreateVertexBuffer(&verts[0], sizeof(verts));
	}

	void CreateVertexBuffer(const void* data, unsigned int sizeInBytes)
	{
		GvkHelper::create_buffer(physicalDevice, device, sizeInBytes,VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, 
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
			&vertexHandle, &vertexData);
		GvkHelper::write_to_buffer(device, vertexData, data, sizeInBytes); // Transfer triangle data to the vertex buffer. (staging would be prefered here)
	}

	//part 2d
	void initializeUniformBuffer()
	{
		unsigned int bufferSize = sizeof(shaderVars);  //size of the uniform data

		//gets the number of active frames
		uint32_t imageCount;
		vlk.GetSwapchainImageCount(imageCount);

		//resizes the vectors for the uniform buffers for each frame
		uniformBufferHandle.resize(imageCount);
		uniformBufferData.resize(imageCount);

		for (size_t i = 0; i < imageCount; i++) //loops through each active frame and creates a buffer for each
		{ 
			GvkHelper::create_buffer(physicalDevice, device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, 
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &uniformBufferHandle[i], &uniformBufferData[i]);
			GvkHelper::write_to_buffer(device, uniformBufferData[i], &shaderVarsUniformBuffer, bufferSize);
		}
	}

	//part 2f
	void initializeDescriptorPool()
	{
		VkDescriptorPoolSize poolSize = {};
		poolSize.descriptorCount = uniformBufferHandle.size();
		poolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;

		VkDescriptorPoolCreateInfo descriptorPoolInfo = {};
		descriptorPoolInfo.flags = 0;
		descriptorPoolInfo.maxSets = uniformBufferHandle.size();
		descriptorPoolInfo.pNext = nullptr;
		descriptorPoolInfo.poolSizeCount = 1;
		descriptorPoolInfo.pPoolSizes = &poolSize;
		descriptorPoolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;

		vkCreateDescriptorPool(device, &descriptorPoolInfo, nullptr, &descriptorPool);
	}

	//part 2g
	void initializeDescriptorSets()
	{
		VkDescriptorSetAllocateInfo descriptorAllocateInfo = {};
		descriptorAllocateInfo.descriptorPool = descriptorPool;
		descriptorAllocateInfo.descriptorSetCount = 1;
		descriptorAllocateInfo.pNext = nullptr;
		descriptorAllocateInfo.pSetLayouts = &descriptorSetLayout;
		descriptorAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;

		descriptorSets.resize(uniformBufferHandle.size());

		for (int i = 0; i < uniformBufferData.size(); i++)
		{
			vkAllocateDescriptorSets(device, &descriptorAllocateInfo, &descriptorSets[i]);
		}
	}

	//part 2h
	void linkDescriptorSetUniformBuffer()
	{
		for (int i = 0; i < uniformBufferData.size(); i++)
		{
			VkDescriptorBufferInfo descriptorBuffer = {};
			descriptorBuffer.buffer = uniformBufferHandle[i];
			descriptorBuffer.offset = 0;
			descriptorBuffer.range = sizeof(shaderVars);

			VkWriteDescriptorSet writeDescriptor = {};
			writeDescriptor.descriptorCount = 1;
			writeDescriptor.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
			writeDescriptor.dstArrayElement = 0;
			writeDescriptor.dstBinding = 0;
			writeDescriptor.dstSet = descriptorSets[i];
			writeDescriptor.pBufferInfo = &descriptorBuffer;
			writeDescriptor.pImageInfo = nullptr;
			writeDescriptor.pNext = nullptr;
			writeDescriptor.pTexelBufferView = nullptr;
			writeDescriptor.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;

			vkUpdateDescriptorSets(device, 1, &writeDescriptor, 0, nullptr);
		}
	}

	void CompileShaders()
	{
		// Intialize runtime shader compiler HLSL -> SPIRV
		shaderc_compiler_t compiler = shaderc_compiler_initialize();
		shaderc_compile_options_t options = CreateCompileOptions();

		CompileVertexShader(compiler, options);
		CompileFragmentShader(compiler, options);

		// Free runtime shader compiler resources
		shaderc_compile_options_release(options);
		shaderc_compiler_release(compiler);
	}

	shaderc_compile_options_t CreateCompileOptions()
	{
		shaderc_compile_options_t retval = shaderc_compile_options_initialize();
		shaderc_compile_options_set_source_language(retval, shaderc_source_language_hlsl);
		shaderc_compile_options_set_invert_y(retval, true);	// TODO: Part 3e
#ifndef NDEBUG
		shaderc_compile_options_set_generate_debug_info(retval);
#endif
		return retval;
	}

	void CompileVertexShader(const shaderc_compiler_t& compiler, const shaderc_compile_options_t& options)
	{
		std::string vertexShaderSource = ReadFileIntoString("../VertexShader.hlsl");
		
		shaderc_compilation_result_t result = shaderc_compile_into_spv( // compile
			compiler, vertexShaderSource.c_str(), vertexShaderSource.length(),
			shaderc_vertex_shader, "main.vert", "main", options);

		if (shaderc_result_get_compilation_status(result) != shaderc_compilation_status_success) // errors?
		{
			PrintLabeledDebugString("Vertex Shader Errors: \n", shaderc_result_get_error_message(result));
			abort(); //Vertex shader failed to compile! 
			return;
		}

		GvkHelper::create_shader_module(device, shaderc_result_get_length(result), // load into Vulkan
			(char*)shaderc_result_get_bytes(result), &vertexShader);

		shaderc_result_release(result); // done
	}

	void CompileFragmentShader(const shaderc_compiler_t& compiler, const shaderc_compile_options_t& options)
	{
		std::string fragmentShaderSource = ReadFileIntoString("../FragmentShader.hlsl");

		shaderc_compilation_result_t result = shaderc_compile_into_spv( // compile
			compiler, fragmentShaderSource.c_str(), fragmentShaderSource.length(),
			shaderc_fragment_shader, "main.frag", "main", options);

		if (shaderc_result_get_compilation_status(result) != shaderc_compilation_status_success) // errors?
		{
			PrintLabeledDebugString("Fragment Shader Errors: \n", shaderc_result_get_error_message(result));
			abort(); //Fragment shader failed to compile! 
			return;
		}

		GvkHelper::create_shader_module(device, shaderc_result_get_length(result), // load into Vulkan
			(char*)shaderc_result_get_bytes(result), &fragmentShader);

		shaderc_result_release(result); // done
	}

	// Create Pipeline & Layout (Thanks Tiny!)
	void InitializeGraphicsPipeline()
	{
		VkPipelineShaderStageCreateInfo stage_create_info[2] = {};	

		// Create Stage Info for Vertex Shader
		stage_create_info[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		stage_create_info[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
		stage_create_info[0].module = vertexShader;
		stage_create_info[0].pName = "main";

		// Create Stage Info for Fragment Shader
		stage_create_info[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		stage_create_info[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
		stage_create_info[1].module = fragmentShader;
		stage_create_info[1].pName = "main";


		VkPipelineInputAssemblyStateCreateInfo assembly_create_info = CreateVkPipelineInputAssemblyStateCreateInfo();
		VkVertexInputBindingDescription vertex_binding_description = CreateVkVertexInputBindingDescription();

		// TODO: Part 1c
		VkVertexInputAttributeDescription vertex_attribute_descriptions[1];
		vertex_attribute_descriptions[0].binding = 0;
		vertex_attribute_descriptions[0].location = 0;
		vertex_attribute_descriptions[0].format = VK_FORMAT_R32G32B32A32_SFLOAT;
		vertex_attribute_descriptions[0].offset = 0;

		VkPipelineVertexInputStateCreateInfo input_vertex_info = CreateVkPipelineVertexInputStateCreateInfo(&vertex_binding_description, 1, vertex_attribute_descriptions, 1);
		VkViewport viewport = CreateViewportFromWindowDimensions();
		VkRect2D scissor = CreateScissorFromWindowDimensions();
		VkPipelineViewportStateCreateInfo viewport_create_info = CreateVkPipelineViewportStateCreateInfo(&viewport, 1, &scissor, 1);
		VkPipelineRasterizationStateCreateInfo rasterization_create_info = CreateVkPipelineRasterizationStateCreateInfo();
		VkPipelineMultisampleStateCreateInfo multisample_create_info = CreateVkPipelineMultisampleStateCreateInfo();
		VkPipelineDepthStencilStateCreateInfo depth_stencil_create_info = CreateVkPipelineDepthStencilStateCreateInfo();
		VkPipelineColorBlendAttachmentState color_blend_attachment_state = CreateVkPipelineColorBlendAttachmentState();
		VkPipelineColorBlendStateCreateInfo color_blend_create_info = CreateVkPipelineColorBlendStateCreateInfo(&color_blend_attachment_state, 1);

		VkDynamicState dynamic_states[2] = 
		{
			// By setting these we do not need to re-create the pipeline on Resize
			VK_DYNAMIC_STATE_VIEWPORT, 
			VK_DYNAMIC_STATE_SCISSOR
		};

		VkPipelineDynamicStateCreateInfo dynamic_create_info = CreateVkPipelineDynamicStateCreateInfo(dynamic_states, 2);

		CreatePipelineLayout();

		// Pipeline State... (FINALLY) 
		VkGraphicsPipelineCreateInfo pipeline_create_info = {};
		pipeline_create_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
		pipeline_create_info.stageCount = 2;
		pipeline_create_info.pStages = stage_create_info;
		pipeline_create_info.pInputAssemblyState = &assembly_create_info;
		pipeline_create_info.pVertexInputState = &input_vertex_info;
		pipeline_create_info.pViewportState = &viewport_create_info;
		pipeline_create_info.pRasterizationState = &rasterization_create_info;
		pipeline_create_info.pMultisampleState = &multisample_create_info;
		pipeline_create_info.pDepthStencilState = &depth_stencil_create_info;
		pipeline_create_info.pColorBlendState = &color_blend_create_info;
		pipeline_create_info.pDynamicState = &dynamic_create_info;
		pipeline_create_info.layout = pipelineLayout;
		pipeline_create_info.renderPass = renderPass;
		pipeline_create_info.subpass = 0;
		pipeline_create_info.basePipelineHandle = VK_NULL_HANDLE;
		
		vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipeline_create_info, nullptr, &pipeline);
	}

	VkPipelineShaderStageCreateInfo CreateVertexShaderStageCreateInfo()
	{
		VkPipelineShaderStageCreateInfo retval;
		retval.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		retval.stage = VK_SHADER_STAGE_VERTEX_BIT;
		retval.module = vertexShader;
		retval.pName = "main";
		return retval;
	}

	VkPipelineInputAssemblyStateCreateInfo CreateVkPipelineInputAssemblyStateCreateInfo()
	{
		VkPipelineInputAssemblyStateCreateInfo retval = {};
		retval.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
		retval.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST; // TODO: Part 1b
		retval.primitiveRestartEnable = false;
		return retval;
	}

	VkVertexInputBindingDescription CreateVkVertexInputBindingDescription()
	{
		VkVertexInputBindingDescription retval = {};
		retval.binding = 0;
		retval.stride = sizeof(vertex); 	//TODO: Part 1c
		retval.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
		return retval;
	}

	VkPipelineVertexInputStateCreateInfo CreateVkPipelineVertexInputStateCreateInfo(
		VkVertexInputBindingDescription* inputBindingDescriptions, unsigned int bindingCount,
		VkVertexInputAttributeDescription* vertexAttributeDescriptions, unsigned int attributeCount)
	{
		VkPipelineVertexInputStateCreateInfo retval = {};
		retval.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
		retval.vertexBindingDescriptionCount = bindingCount;
		retval.pVertexBindingDescriptions = inputBindingDescriptions;
		retval.vertexAttributeDescriptionCount = attributeCount;
		retval.pVertexAttributeDescriptions = vertexAttributeDescriptions;
		return retval;
	}

	VkViewport CreateViewportFromWindowDimensions()
	{
		VkViewport retval = {};
		retval.x = 0;
		retval.y = 0;
		retval.width = static_cast<float>(windowWidth);
		retval.height = static_cast<float>(windowHeight);
		retval.minDepth = 0;
		retval.maxDepth = 1;
		return retval;
	}

	VkRect2D CreateScissorFromWindowDimensions()
	{
		VkRect2D retval = {};
		retval.offset.x = 0;
		retval.offset.y = 0;
		retval.extent.width = windowWidth;
		retval.extent.height = windowHeight;
		return retval;
	}

	VkPipelineViewportStateCreateInfo CreateVkPipelineViewportStateCreateInfo(VkViewport* viewports, unsigned int viewportCount, VkRect2D* scissors, unsigned int scissorCount)
	{
		VkPipelineViewportStateCreateInfo retval = {};
		retval.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
		retval.viewportCount = viewportCount;
		retval.pViewports = viewports;
		retval.scissorCount = scissorCount;
		retval.pScissors = scissors;
		return retval;
	}

	VkPipelineRasterizationStateCreateInfo CreateVkPipelineRasterizationStateCreateInfo()
	{
		VkPipelineRasterizationStateCreateInfo retval = {};
		retval.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
		retval.rasterizerDiscardEnable = VK_FALSE;
		retval.polygonMode = VK_POLYGON_MODE_FILL;
		retval.lineWidth = 1.0f;
		retval.cullMode = VK_CULL_MODE_BACK_BIT;
		retval.frontFace = VK_FRONT_FACE_CLOCKWISE;
		retval.depthClampEnable = VK_FALSE;
		retval.depthBiasEnable = VK_FALSE;
		retval.depthBiasClamp = 0.0f;
		retval.depthBiasConstantFactor = 0.0f;
		retval.depthBiasSlopeFactor = 0.0f;
		return retval;
	}

	VkPipelineMultisampleStateCreateInfo CreateVkPipelineMultisampleStateCreateInfo()
	{
		VkPipelineMultisampleStateCreateInfo retval = {};
		retval.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
		retval.sampleShadingEnable = VK_FALSE;
		retval.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
		retval.minSampleShading = 1.0f;
		retval.pSampleMask = VK_NULL_HANDLE;
		retval.alphaToCoverageEnable = VK_FALSE;
		retval.alphaToOneEnable = VK_FALSE;
		return retval;
	}

	VkPipelineDepthStencilStateCreateInfo CreateVkPipelineDepthStencilStateCreateInfo()
	{
		VkPipelineDepthStencilStateCreateInfo retval = {};
		retval.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
		retval.depthTestEnable = VK_TRUE;
		retval.depthWriteEnable = VK_TRUE;
		retval.depthCompareOp = VK_COMPARE_OP_LESS;
		retval.depthBoundsTestEnable = VK_FALSE;
		retval.minDepthBounds = 0.0f;
		retval.maxDepthBounds = 1.0f;
		retval.stencilTestEnable = VK_FALSE;
		return retval;
	}

	VkPipelineColorBlendAttachmentState CreateVkPipelineColorBlendAttachmentState()
	{
		VkPipelineColorBlendAttachmentState retval = {};
		retval.colorWriteMask = 0xF;
		retval.blendEnable = VK_FALSE;
		retval.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_COLOR;
		retval.dstColorBlendFactor = VK_BLEND_FACTOR_DST_COLOR;
		retval.colorBlendOp = VK_BLEND_OP_ADD;
		retval.srcAlphaBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
		retval.dstAlphaBlendFactor = VK_BLEND_FACTOR_DST_ALPHA;
		retval.alphaBlendOp = VK_BLEND_OP_ADD;
		return retval;
	}

	VkPipelineColorBlendStateCreateInfo CreateVkPipelineColorBlendStateCreateInfo(VkPipelineColorBlendAttachmentState* attachments, unsigned int attachmentCount)
	{
		VkPipelineColorBlendStateCreateInfo retval = {};
		retval.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
		retval.logicOpEnable = VK_FALSE;
		retval.logicOp = VK_LOGIC_OP_COPY;
		retval.attachmentCount = attachmentCount;
		retval.pAttachments = attachments;
		retval.blendConstants[0] = 0.0f;
		retval.blendConstants[1] = 0.0f;
		retval.blendConstants[2] = 0.0f;
		retval.blendConstants[3] = 0.0f;
		return retval;
	}

	VkPipelineDynamicStateCreateInfo CreateVkPipelineDynamicStateCreateInfo(VkDynamicState* dynamicStates, unsigned int dynamicStateCount)
	{
		VkPipelineDynamicStateCreateInfo retval = {};
		retval.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
		retval.dynamicStateCount = dynamicStateCount;
		retval.pDynamicStates = dynamicStates;
		return retval;
	}

	void CreatePipelineLayout()
	{
		// TODO: Part 2e
		
		VkPipelineLayoutCreateInfo pipeline_layout_create_info = {};
		pipeline_layout_create_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
		pipeline_layout_create_info.setLayoutCount = 1; // TODO: Part 2e
		pipeline_layout_create_info.pSetLayouts = &descriptorSetLayout; // TODO: Part 2e
		pipeline_layout_create_info.pushConstantRangeCount = 0; 

		vkCreatePipelineLayout(device, &pipeline_layout_create_info, nullptr, &pipelineLayout);
	}

	void BindShutdownCallback()
	{
		// GVulkanSurface will inform us when to release any allocated resources
		shutdown.Create(vlk, [&]() {
			if (+shutdown.Find(GW::GRAPHICS::GVulkanSurface::Events::RELEASE_RESOURCES, true)) {
				CleanUp(); // unlike D3D we must be careful about destroy timing
			}
			});
	}

public:
	void Render()
	{
		VkCommandBuffer commandBuffer = GetCurrentCommandBuffer();
		// TODO: Part 4x
		SetUpPipeline(commandBuffer);

		// TODO: Part 2i // TODO: Part 4y;
		uint32_t activeImage;
		vlk.GetSwapchainCurrentImage(activeImage);
		vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets[activeImage], 0, 0);

		// TODO: Part 3g
		vkCmdDraw(commandBuffer, 104, 6, 0, 0); // TODO: Part 1b 
	}

	// TODO: Part 4b
	void updateCamera()
	{
		float elapsedTime = std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - startTime).count();

		// TODO: Part 4c
		GW::MATH::GMATRIXF viewCopy {};
		interfaceProxy.InverseF(viewMatrix, viewCopy);

		uint32_t currentImage;
		vlk.GetSwapchainCurrentImage(currentImage);

		// TODO: Part 4d
		float yChange = 0.0f;
		float states[6] = { 0, 0, 0, 0, 0, 0 };
		const float cameraSpeed = 0.0003f;

		input.GetState(G_KEY_SPACE, states[0]);
		input.GetState(G_KEY_LEFTSHIFT, states[1]);
		controller.GetState(0, G_RIGHT_TRIGGER_AXIS, states[2]);
		controller.GetState(0, G_LEFT_TRIGGER_AXIS, states[3]);

		yChange = states[0] - states[1] + states[2] - states[3];
		viewCopy.row4.y += static_cast<float>(yChange * cameraSpeed * elapsedTime);

		// TODO: Part 4e
		const float perFrameSpeed = cameraSpeed * elapsedTime;
		input.GetState(G_KEY_W, states[0]);
		input.GetState(G_KEY_S, states[1]);
		input.GetState(G_KEY_A, states[2]);
		input.GetState(G_KEY_D, states[3]);
		controller.GetState(0, G_LY_AXIS, states[4]);
		controller.GetState(0, G_LX_AXIS, states[5]);
		float zChange = states[0] - states[1] + states[4];
		float xChange = states[3] - states[2] + states[5];
		GW::MATH::GVECTORF translate{xChange * perFrameSpeed, 0, zChange * perFrameSpeed};
		interfaceProxy.TranslateLocalF(viewCopy, translate, viewCopy);

		// TODO: Part 4f
		unsigned int height;
		win.GetClientHeight(height);
		bool focused;
		win.IsFocus(focused);
		if (input.GetMouseDelta(states[0], states[1]) != GW::GReturn::SUCCESS || !focused)
		{
			states[0] = states[1] = 0;
		}
		controller.GetState(0, G_RY_AXIS, states[2]);
		controller.GetState(0, G_RX_AXIS, states[3]);

		float thumbSpeed = G_PI * elapsedTime;
		float totalPitch = G_PI / 2 * states[1] / height + states[2] * -thumbSpeed;
		GW::MATH::GMATRIXF pitchMatrix{};
		GW::MATH::GMATRIXF identity = GW::MATH::GIdentityMatrixF;
		interfaceProxy.RotateXLocalF(identity, totalPitch, pitchMatrix);
		interfaceProxy.MultiplyMatrixF(pitchMatrix, viewCopy, viewCopy);

		// TODO: Part 4g
		unsigned int width;
		win.GetClientWidth(width);
		float ar = width / static_cast<float>(height);
		float yaw = G_PI / 2 * ar * states[0] / width + states[3] * thumbSpeed;
		GW::MATH::GMATRIXF yawMatrix;
		interfaceProxy.RotateYLocalF(identity, yaw, yawMatrix);
		GW::MATH::GVECTORF pos = viewCopy.row4;
		interfaceProxy.MultiplyMatrixF(viewCopy, yawMatrix, viewCopy);
		viewCopy.row4 = pos;

		interfaceProxy.InverseF(viewCopy, viewMatrix);
		shaderVarsUniformBuffer.viewMatrix = viewMatrix;
		GvkHelper::write_to_buffer(device, uniformBufferData[currentImage], &shaderVarsUniformBuffer, sizeof(shaderVars));
	}

private:
	VkCommandBuffer GetCurrentCommandBuffer()
	{
		unsigned int currentBuffer;
		vlk.GetSwapchainCurrentImage(currentBuffer);

		VkCommandBuffer commandBuffer;
		vlk.GetCommandBuffer(currentBuffer, (void**)&commandBuffer);
		return commandBuffer;
	}

	void SetUpPipeline(VkCommandBuffer& commandBuffer)
	{
		UpdateWindowDimensions();
		SetViewport(commandBuffer);
		SetScissor(commandBuffer);
		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
		BindVertexBuffers(commandBuffer);
	}

	void SetViewport(const VkCommandBuffer& commandBuffer)
	{
		VkViewport viewport = CreateViewportFromWindowDimensions();
		vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
	}

	void SetScissor(const VkCommandBuffer& commandBuffer)
	{
		VkRect2D scissor = CreateScissorFromWindowDimensions();
		vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
	}

	void BindVertexBuffers(VkCommandBuffer& commandBuffer)
	{
		VkDeviceSize offsets[] = { 0 };
		vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vertexHandle, offsets);
	}


	//Cleanup callback function (passed to VKSurface, will be called when the pipeline shuts down)
	void CleanUp()
	{
		// wait till everything has completed
		vkDeviceWaitIdle(device);

		// Release allocated buffers, shaders & pipeline
		vkDestroyBuffer(device, vertexHandle, nullptr);
		vkFreeMemory(device, vertexData, nullptr);
		// TODO: Part 2d
		for (int i = 0; i < uniformBufferHandle.size(); i++)
		{
			vkDestroyBuffer(device, uniformBufferHandle[i], nullptr);

			vkFreeMemory(device, uniformBufferData[i], nullptr);
		}
		uniformBufferHandle.clear();
		uniformBufferData.clear();

		// TODO: Part 2f
		vkDestroyDescriptorSetLayout(device, descriptorSetLayout, nullptr);
		vkDestroyDescriptorPool(device, descriptorPool, nullptr);

		vkDestroyShaderModule(device, vertexShader, nullptr);
		vkDestroyShaderModule(device, fragmentShader, nullptr);
		vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
		vkDestroyPipeline(device, pipeline, nullptr);
	}
};