// Copyright (c) 2023 Janusch Patas.
// All rights reserved. Derived from 3D Gaussian Splatting for Real-Time Radiance Field Rendering software by Inria and MPII.
#include "gs/rasterizer.cuh"

// _RasterizeGaussians
torch::autograd::tensor_list _RasterizeGaussians::forward(
    torch::autograd::AutogradContext* ctx,
    torch::Tensor means3D,
    torch::Tensor means2D,
    torch::Tensor sh,
    torch::Tensor colors_precomp,
    torch::Tensor opacities,
    torch::Tensor scales,
    torch::Tensor rotations,
    torch::Tensor cov3Ds_precomp,
    torch::Tensor image_height,
    torch::Tensor image_width,
    torch::Tensor tanfovx,
    torch::Tensor tanfovy,
    torch::Tensor bg,
    torch::Tensor scale_modifier,
    torch::Tensor viewmatrix,
    torch::Tensor projmatrix,
    torch::Tensor sh_degree,
    torch::Tensor camera_center,
    torch::Tensor prefiltered) {
  int image_height_val = image_height.item<int>();
  int image_width_val = image_width.item<int>();
  float tanfovx_val = tanfovx.item<float>();
  float tanfovy_val = tanfovy.item<float>();
  float scale_modifier_val = scale_modifier.item<float>();
  int sh_degree_val = sh_degree.item<int>();
  bool prefiltered_val = prefiltered.item<bool>();

  auto [num_rendered, color, out_depth, out_acc, radii, geomBuffer, binningBuffer, imgBuffer] = RasterizeGaussiansCUDA(
      bg,
      means3D,
      colors_precomp,
      opacities,
      scales,
      rotations,
      scale_modifier_val,
      cov3Ds_precomp,
      viewmatrix,
      projmatrix,
      tanfovx_val,
      tanfovy_val,
      image_height_val,
      image_width_val,
      sh,
      sh_degree_val,
      camera_center,
      prefiltered_val,
      false);

  // Only save tensors required for the backward pass. Color, radii and
  // precomputed covariance can be reconstructed from the geometry buffers and
  // therefore do not need to be stored here. The camera center is later
  // recomputed from the view matrix and other scalar parameters like image
  // dimensions or the prefiltered flag are unnecessary. This reduces the
  // memory footprint during training.
  ctx->save_for_backward({means3D, scales, rotations, sh, geomBuffer, binningBuffer, imgBuffer});
  ctx->saved_data["num_rendered"] = num_rendered;
  ctx->saved_data["background"] = bg;
  ctx->saved_data["scale_modifier"] = scale_modifier_val;
  ctx->saved_data["viewmatrix"] = viewmatrix;
  ctx->saved_data["projmatrix"] = projmatrix;
  ctx->saved_data["tanfovx"] = tanfovx_val;
  ctx->saved_data["tanfovy"] = tanfovy_val;
  ctx->saved_data["sh_degree"] = sh_degree_val;
  return {color, radii, out_depth, out_acc};
}

torch::autograd::tensor_list _RasterizeGaussians::backward(
    torch::autograd::AutogradContext* ctx,
    torch::autograd::tensor_list grad_outputs) {
  auto grad_out_color = grad_outputs[0];
  auto grad_out_radii = grad_outputs[1];
  auto grad_out_depth = grad_outputs[2];
  auto grad_acc = grad_outputs[3];

  // Radii and depth gradients are currently unused but are retained to keep the
  // signature consistent with the forward pass.
  (void)grad_out_radii;
  (void)grad_out_depth;

  int num_rendered = ctx->saved_data["num_rendered"].to<int>();
  auto saved = ctx->get_saved_variables();
  auto means3D = saved[0];
  auto scales = saved[1];
  auto rotations = saved[2];
  auto sh = saved[3];
  auto geomBuffer = saved[4];
  auto binningBuffer = saved[5];
  auto imgBuffer = saved[6];

  // Retrieve matrices used for recomputation of auxiliary parameters.
  auto viewmatrix = ctx->saved_data["viewmatrix"].to<torch::Tensor>();
  auto projmatrix = ctx->saved_data["projmatrix"].to<torch::Tensor>();

  // Recompute camera position from the view matrix instead of storing it
  // explicitly in the autograd context.
  auto camera_center = viewmatrix.inverse()[3].slice(0, 0, 3);

  // Recompute or retrieve tensors that are stored inside the geometry buffer
  // instead of saving them explicitly during the forward pass.
  torch::Tensor radii;            // use internal radii from geomBuffer
  torch::Tensor colors_precomp;   // geomBuffer contains final RGB values
  torch::Tensor cov3Ds_precomp;   // covariance is also stored in geomBuffer

  auto
      [grad_means2D,
       grad_colors_precomp,
       grad_opacities,
       grad_means3D,
       grad_cov3Ds_precomp,
       grad_sh,
       grad_scales,
       grad_rotations] =
          RasterizeGaussiansBackwardCUDA(
              ctx->saved_data["background"].to<torch::Tensor>(),
              means3D,
              radii,
              colors_precomp,
              scales,
              rotations,
              ctx->saved_data["scale_modifier"].to<float>(),
              cov3Ds_precomp,
              viewmatrix,
              projmatrix,
              ctx->saved_data["tanfovx"].to<float>(),
              ctx->saved_data["tanfovy"].to<float>(),
              grad_out_color,
              grad_acc,
              sh,
              ctx->saved_data["sh_degree"].to<int>(),
              camera_center,
              geomBuffer,
              num_rendered,
              binningBuffer,
              imgBuffer,
              false);

  // return gradients for all inputs, 19 in total. :D
  return {
      grad_means3D,
      grad_means2D,
      grad_sh,
      grad_colors_precomp,
      grad_opacities,
      grad_scales,
      grad_rotations,
      grad_cov3Ds_precomp,
      torch::Tensor(),  // from here placeholder, not used: #forwards args = #backwards args.
      torch::Tensor(),
      torch::Tensor(),
      torch::Tensor(),
      torch::Tensor(),
      torch::Tensor(),
      torch::Tensor(),
      torch::Tensor(),
      torch::Tensor(),
      torch::Tensor(),
      torch::Tensor()};
}

// GaussianRasterizer
std::tuple<torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor> GaussianRasterizer::forward(
    torch::Tensor means3D,
    torch::Tensor means2D,
    torch::Tensor opacities,
    torch::Tensor shs,
    torch::Tensor colors_precomp,
    torch::Tensor scales,
    torch::Tensor rotations,
    torch::Tensor cov3D_precomp) {
  if ((shs.defined() && colors_precomp.defined()) || (!shs.defined() && !colors_precomp.defined())) {
    throw std::invalid_argument("Please provide exactly one of either SHs or precomputed colors!");
  }
  if (((scales.defined() || rotations.defined()) && cov3D_precomp.defined()) ||
      (!scales.defined() && !rotations.defined() && !cov3D_precomp.defined())) {
    throw std::invalid_argument(
        "Please provide exactly one of either scale/rotation pair or "
        "precomputed 3D covariance!");
  }

  // Check if tensors are undefined, and if so, initialize them
  torch::Device device = torch::kCUDA;
  if (!shs.defined()) {
    std::cerr << "No colors. Exiting program." << std::endl;
    exit(1);
    // shs = torch::empty({0}, device);
  }
  if (!colors_precomp.defined()) {
    colors_precomp = torch::empty({0}, device);
  }
  if (!scales.defined()) {
    std::cerr << "No scales. Exiting program." << std::endl;
    exit(1);
    // scales = torch::empty({0}, device);
  }
  if (!rotations.defined()) {
    std::cerr << "No rotations. Exiting program." << std::endl;
    exit(1);
    // rotations = torch::empty({0}, device);
  }
  if (!cov3D_precomp.defined()) {
    cov3D_precomp = torch::empty({0}, device);
  }

  auto result = this->rasterize_gaussians(
      means3D, means2D, shs, colors_precomp, opacities, scales, rotations, cov3D_precomp, raster_settings_);

  return {result[0], result[1], result[2], result[3]};
}

torch::Tensor GaussianRasterizer::mark_visible(torch::Tensor positions) {
  torch::NoGradGuard no_grad;
  auto visible = markVisible(positions, raster_settings_.viewmatrix, raster_settings_.projmatrix);

  return visible;
}

torch::autograd::tensor_list GaussianRasterizer::rasterize_gaussians(
    torch::Tensor means3D,
    torch::Tensor means2D,
    torch::Tensor sh,
    torch::Tensor colors_precomp,
    torch::Tensor opacities,
    torch::Tensor scales,
    torch::Tensor rotations,
    torch::Tensor cov3Ds_precomp,
    GaussianRasterizationSettings raster_settings) {
  torch::Device device = torch::kCUDA;
  auto image_height = torch::tensor(raster_settings.image_height, device);
  auto image_width = torch::tensor(raster_settings.image_width, device);
  auto tanfovx = torch::tensor(raster_settings.tanfovx, device);
  auto tanfovy = torch::tensor(raster_settings.tanfovy, device);

  if (!raster_settings.bg.is_cuda()) {
    raster_settings.bg = raster_settings.bg.to(device);
  }
  auto scale_modifier = torch::tensor(raster_settings.scale_modifier, device);

  if (!raster_settings.viewmatrix.is_cuda()) {
    raster_settings.viewmatrix = raster_settings.viewmatrix.to(device);
  }
  if (!raster_settings.projmatrix.is_cuda()) {
    raster_settings.projmatrix = raster_settings.projmatrix.to(device);
  }

  auto sh_degree = torch::tensor(raster_settings.sh_degree, device);

  if (!raster_settings.camera_center.is_cuda()) {
    raster_settings.camera_center = raster_settings.camera_center.to(device);
  }
  auto prefiltered = torch::tensor(raster_settings.prefiltered, device);

  if (means2D.device() != device) {
    means2D = means2D.to(device);
  }
  if (means3D.device() != device) {
    means3D = means3D.to(device);
  }
  if (sh.device() != device) {
    sh = sh.to(device);
  }
  if (colors_precomp.device() != device) {
    colors_precomp = colors_precomp.to(device);
  }
  if (opacities.device() != device) {
    opacities = opacities.to(device);
  }
  if (scales.device() != device) {
    scales = scales.to(device);
  }
  if (rotations.device() != device) {
    rotations = rotations.to(device);
  }
  if (cov3Ds_precomp.device() != device) {
    cov3Ds_precomp = cov3Ds_precomp.to(device);
  }

  return _RasterizeGaussians::apply(
      means3D,
      means2D,
      sh,
      colors_precomp,
      opacities,
      scales,
      rotations,
      cov3Ds_precomp,
      image_height,
      image_width,
      tanfovx,
      tanfovy,
      raster_settings.bg,
      scale_modifier,
      raster_settings.viewmatrix,
      raster_settings.projmatrix,
      sh_degree,
      raster_settings.camera_center,
      prefiltered);
}
